#!/usr/bin/env python3
import numpy as np
import os
from functools import partial

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointField
from tf_transformations import quaternion_matrix
import tf2_ros
import message_filters
from cv_bridge import CvBridge
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout as MAL
from std_msgs.msg import MultiArrayDimension as MAD
from elevation_mapping_cupy import ElevationMap, Parameter


PDC_DATATYPE = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


class ElevationMappingNode(Node):
    def __init__(self):
        super().__init__(
            "elevation_mapping_node",
            automatically_declare_parameters_from_overrides=True,
            allow_undeclared_parameters=True,
        )

        self.root = get_package_share_directory("elevation_mapping_cupy")
        weight_file = os.path.join(self.root, "config/core/weights.dat")
        plugin_config_file = os.path.join(self.root, "config/core/plugin_config.yaml")

        self.param = Parameter(
            use_chainer=False,
            weight_file=weight_file,
            plugin_config_file=plugin_config_file,
        )

        self.initialize_ros()
        self.set_param_values_from_ros()

        # 用 YAML 中读到的 subscriber 配置覆盖
        self.param.subscriber_cfg = self.my_subscribers

        self.initialize_elevation_mapping()
        self.register_subscribers()
        self.register_publishers()
        self.register_timers()

        self._last_t = None

        # body 系局部高程图发布参数
        self.body_map_length = 4.0
        self.body_map_layer = "inpaint"
        self.body_map_topic = f"/{self.get_name()}/body_elevation_map"

        self._body_map_pub = self.create_publisher(
            GridMap,
            self.body_map_topic,
            10,
        )

        self.body_map_timer = self.create_timer(
            0.1,
            lambda: self.publish_map_in_body(
                layer_name=self.body_map_layer,
                local_length=self.body_map_length,
            ),
        )

    def initialize_elevation_mapping(self) -> None:
        self.param.update()
        self._pointcloud_process_counter = 0
        self._image_process_counter = 0
        self._map = ElevationMap(self.param)
        self._map_data = np.zeros(
            (self._map.cell_n - 2, self._map.cell_n - 2), dtype=np.float32
        )
        self.get_logger().info(
            f"Initialized map with length: {self._map.map_length}, "
            f"resolution: {self._map.resolution}, cells: {self._map.cell_n}"
        )

        self._map_q = None
        self._map_t = None

    def initialize_ros(self) -> None:
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.get_ros_params()

    def get_ros_params(self) -> None:
        self.use_chainer = self.get_parameter("use_chainer").get_parameter_value().bool_value
        self.weight_file = self.get_parameter("weight_file").get_parameter_value().string_value
        self.plugin_config_file = self.get_parameter("plugin_config_file").get_parameter_value().string_value
        self.initialize_frame_id = self.get_parameter("initialize_frame_id").get_parameter_value().string_value
        self.initialize_tf_offset = self.get_parameter("initialize_tf_offset").get_parameter_value().double_array_value
        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.corrected_map_frame = self.get_parameter("corrected_map_frame").get_parameter_value().string_value
        self.initialize_method = self.get_parameter("initialize_method").get_parameter_value().string_value
        self.position_lowpass_alpha = self.get_parameter("position_lowpass_alpha").get_parameter_value().double_value
        self.orientation_lowpass_alpha = self.get_parameter("orientation_lowpass_alpha").get_parameter_value().double_value
        self.recordable_fps = self.get_parameter("recordable_fps").get_parameter_value().double_value
        self.update_variance_fps = self.get_parameter("update_variance_fps").get_parameter_value().double_value
        self.time_interval = self.get_parameter("time_interval").get_parameter_value().double_value
        self.update_pose_fps = self.get_parameter("update_pose_fps").get_parameter_value().double_value
        self.initialize_tf_grid_size = self.get_parameter("initialize_tf_grid_size").get_parameter_value().double_value
        self.map_acquire_fps = self.get_parameter("map_acquire_fps").get_parameter_value().double_value
        self.publish_statistics_fps = self.get_parameter("publish_statistics_fps").get_parameter_value().double_value
        self.enable_pointcloud_publishing = self.get_parameter("enable_pointcloud_publishing").get_parameter_value().bool_value
        self.enable_normal_arrow_publishing = self.get_parameter("enable_normal_arrow_publishing").get_parameter_value().bool_value
        self.enable_drift_corrected_TF_publishing = self.get_parameter("enable_drift_corrected_TF_publishing").get_parameter_value().bool_value
        self.use_initializer_at_start = self.get_parameter("use_initializer_at_start").get_parameter_value().bool_value

        subscribers_params = self.get_parameters_by_prefix("subscribers")
        self.my_subscribers = {}
        for param_name, param_value in subscribers_params.items():
            parts = param_name.split(".")
            if len(parts) >= 2:
                sub_key, sub_param = parts[:2]
                if sub_key not in self.my_subscribers:
                    self.my_subscribers[sub_key] = {}
                self.my_subscribers[sub_key][sub_param] = param_value.value

        publishers_params = self.get_parameters_by_prefix("publishers")
        self.my_publishers = {}
        for param_name, param_value in publishers_params.items():
            parts = param_name.split(".")
            if len(parts) >= 2:
                pub_key, pub_param = parts[:2]
                if pub_key not in self.my_publishers:
                    self.my_publishers[pub_key] = {}
                self.my_publishers[pub_key][pub_param] = param_value.value

    def set_param_values_from_ros(self):
        try:
            self.param.resolution = self.get_parameter("resolution").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.map_length = self.get_parameter("map_length").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.sensor_noise_factor = self.get_parameter("sensor_noise_factor").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.mahalanobis_thresh = self.get_parameter("mahalanobis_thresh").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.outlier_variance = self.get_parameter("outlier_variance").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.drift_compensation_variance_inlier = self.get_parameter(
                "drift_compensation_variance_inler"
            ).get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.max_drift = self.get_parameter("max_drift").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.drift_compensation_alpha = self.get_parameter(
                "drift_compensation_alpha"
            ).get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.time_variance = self.get_parameter("time_variance").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.max_variance = self.get_parameter("max_variance").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.initial_variance = self.get_parameter("initial_variance").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.traversability_inlier = self.get_parameter(
                "traversability_inlier"
            ).get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.dilation_size = self.get_parameter("dilation_size").get_parameter_value().integer_value
        except Exception:
            pass
        try:
            self.param.wall_num_thresh = self.get_parameter("wall_num_thresh").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.min_height_drift_cnt = self.get_parameter(
                "min_height_drift_cnt"
            ).get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.position_noise_thresh = self.get_parameter(
                "position_noise_thresh"
            ).get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.orientation_noise_thresh = self.get_parameter(
                "orientation_noise_thresh"
            ).get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.min_valid_distance = self.get_parameter(
                "min_valid_distance"
            ).get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.max_height_range = self.get_parameter("max_height_range").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.ramped_height_range_a = self.get_parameter(
                "ramped_height_range_a"
            ).get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.ramped_height_range_b = self.get_parameter(
                "ramped_height_range_b"
            ).get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.ramped_height_range_c = self.get_parameter(
                "ramped_height_range_c"
            ).get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.max_ray_length = self.get_parameter("max_ray_length").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.cleanup_step = self.get_parameter("cleanup_step").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.cleanup_cos_thresh = self.get_parameter(
                "cleanup_cos_thresh"
            ).get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.safe_thresh = self.get_parameter("safe_thresh").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.safe_min_thresh = self.get_parameter("safe_min_thresh").get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.max_unsafe_n = self.get_parameter("max_unsafe_n").get_parameter_value().integer_value
        except Exception:
            pass
        try:
            self.param.overlap_clear_range_xy = self.get_parameter(
                "overlap_clear_range_xy"
            ).get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.overlap_clear_range_z = self.get_parameter(
                "overlap_clear_range_z"
            ).get_parameter_value().double_value
        except Exception:
            pass
        try:
            self.param.enable_edge_sharpen = self.get_parameter(
                "enable_edge_sharpen"
            ).get_parameter_value().bool_value
        except Exception:
            pass
        try:
            self.param.enable_visibility_cleanup = self.get_parameter(
                "enable_visibility_cleanup"
            ).get_parameter_value().bool_value
        except Exception:
            pass
        try:
            self.param.enable_drift_compensation = self.get_parameter(
                "enable_drift_compensation"
            ).get_parameter_value().bool_value
        except Exception:
            pass
        try:
            self.param.enable_overlap_clearance = self.get_parameter(
                "enable_overlap_clearance"
            ).get_parameter_value().bool_value
        except Exception:
            pass
        try:
            self.param.use_only_above_for_upper_bound = self.get_parameter(
                "use_only_above_for_upper_bound"
            ).get_parameter_value().bool_value
        except Exception:
            pass

    def register_subscribers(self) -> None:
        if any(config.get("data_type") == "image" for config in self.my_subscribers.values()):
            self.cv_bridge = CvBridge()

        self.pointcloud_subs = {}
        self.camera_subs = {}
        self.camera_info_subs = {}
        self.image_syncs = {}

        for key, config in self.my_subscribers.items():
            data_type = config.get("data_type")

            if data_type == "image":
                topic_name_camera = config.get("topic_name_camera", "/camera/image")
                topic_name_camera_info = config.get("topic_name_camera_info", "/camera/camera_info")

                camera_sub = message_filters.Subscriber(
                    self,
                    Image,
                    topic_name_camera,
                )
                camera_info_sub = message_filters.Subscriber(
                    self,
                    CameraInfo,
                    topic_name_camera_info,
                )
                image_sync = message_filters.ApproximateTimeSynchronizer(
                    [camera_sub, camera_info_sub],
                    queue_size=10,
                    slop=0.5,
                )
                image_sync.registerCallback(partial(self.image_callback, sub_key=key))

                self.camera_subs[key] = camera_sub
                self.camera_info_subs[key] = camera_info_sub
                self.image_syncs[key] = image_sync

            elif data_type == "pointcloud":
                topic_name = config.get("topic_name", "/pointcloud")
                qos_profile = 10
                subscription = self.create_subscription(
                    PointCloud2,
                    topic_name,
                    partial(self.pointcloud_callback, sub_key=key),
                    qos_profile,
                )
                self.pointcloud_subs[key] = subscription

    def register_publishers(self) -> None:
        self._publishers_dict = {}
        self._publishers_timers = []

        for pub_key, pub_config in self.my_publishers.items():
            topic_name = f"/{self.get_name()}/{pub_key}"
            publisher = self.create_publisher(GridMap, topic_name, 10)
            self._publishers_dict[pub_key] = publisher

            fps = pub_config.get("fps", 1.0)
            timer = self.create_timer(
                1.0 / fps,
                partial(self.publish_map, key=pub_key),
            )
            self._publishers_timers.append(timer)

    def register_timers(self) -> None:
        self.time_pose_update = self.create_timer(0.1, self.pose_update)
        self.timer_variance = self.create_timer(
            1.0 / self.update_variance_fps,
            self.update_variance,
        )
        self.timer_time = self.create_timer(
            self.time_interval,
            self.update_time,
        )

    def publish_map(self, key: str) -> None:
        if self._map_q is None or self._map_t is None:
            return

        gm = GridMap()
        gm.header.frame_id = self.map_frame
        gm.header.stamp = self._last_t if self._last_t is not None else self.get_clock().now().to_msg()

        gm.info.resolution = self._map.resolution
        actual_map_length = (self._map.cell_n - 2) * self._map.resolution
        gm.info.length_x = actual_map_length
        gm.info.length_y = actual_map_length
        gm.info.pose.position.x = self._map_t.x
        gm.info.pose.position.y = self._map_t.y
        gm.info.pose.position.z = 0.0
        gm.info.pose.orientation.w = 1.0
        gm.info.pose.orientation.x = 0.0
        gm.info.pose.orientation.y = 0.0
        gm.info.pose.orientation.z = 0.0

        gm.layers = []
        gm.basic_layers = self.my_publishers[key]["basic_layers"]

        for layer in self.my_publishers[key].get("layers", []):
            gm.layers.append(layer)
            self._map.get_map_with_name_ref(layer, self._map_data)

            map_data_for_gridmap = self._map_data
            arr = Float32MultiArray()
            arr.layout = MAL()

            arr.layout.dim.append(
                MAD(
                    label="column_index",
                    size=map_data_for_gridmap.shape[1],
                    stride=map_data_for_gridmap.shape[0] * map_data_for_gridmap.shape[1],
                )
            )
            arr.layout.dim.append(
                MAD(
                    label="row_index",
                    size=map_data_for_gridmap.shape[0],
                    stride=map_data_for_gridmap.shape[0],
                )
            )
            arr.data = map_data_for_gridmap.flatten().tolist()
            gm.data.append(arr)

        gm.outer_start_index = 0
        gm.inner_start_index = 0
        self._publishers_dict[key].publish(gm)

    def safe_lookup_transform(self, target_frame, source_frame, time):
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                time,
            )
        except tf2_ros.ExtrapolationException:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )

    def image_callback(self, camera_msg: Image, camera_info_msg: CameraInfo, sub_key: str) -> None:
        self._last_t = camera_msg.header.stamp

        try:
            semantic_img = self.cv_bridge.imgmsg_to_cv2(
                camera_msg,
                desired_encoding="passthrough",
            )
        except Exception as e:
            self.get_logger().warn(f"imgmsg_to_cv2 failed: {e}")
            return

        if len(semantic_img.shape) != 2:
            semantic_img = [semantic_img[:, :, k] for k in range(semantic_img.shape[2])]
        else:
            semantic_img = [semantic_img]

        K = np.array(camera_info_msg.k, dtype=np.float32).reshape(3, 3)
        D = np.array(camera_info_msg.d, dtype=np.float32)
        distortion_model = camera_info_msg.distortion_model

        try:
            transform_camera_to_map = self.safe_lookup_transform(
                self.map_frame,
                camera_msg.header.frame_id,
                camera_msg.header.stamp,
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed in image_callback: {e}")
            return

        t = transform_camera_to_map.transform.translation
        q = transform_camera_to_map.transform.rotation
        t_np = np.array([t.x, t.y, t.z], dtype=np.float32)
        R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].astype(np.float32)

        channels = self.param.subscriber_cfg[sub_key].get("channels", [])

        try:
            self._map.input_image(
                semantic_img,
                channels,
                R,
                t_np,
                K,
                D,
                distortion_model,
                camera_info_msg.height,
                camera_info_msg.width,
            )
            self._image_process_counter += 1
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}", exc_info=True)

    def _parse_pointcloud_fields(self, msg: PointCloud2, channels):
        if len(msg.data) == 0 or msg.point_step <= 0:
            return None

        point_count = len(msg.data) // msg.point_step
        raw_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(point_count, msg.point_step)

        pts_final = np.zeros((point_count, len(channels)), dtype=np.float32)
        field_map = {f.name: f for f in msg.fields}

        for i, ch in enumerate(channels):
            if ch not in field_map:
                self.get_logger().warn(f"字段 {ch} 不存在！")
                continue

            field = field_map[ch]
            if field.datatype not in PDC_DATATYPE:
                self.get_logger().warn(
                    f"字段 {ch} 的 datatype={field.datatype} 暂不支持，按 0 处理"
                )
                continue

            np_dtype = PDC_DATATYPE[field.datatype]
            offset = field.offset
            itemsize = np.dtype(np_dtype).itemsize

            try:
                values = raw_data[:, offset:offset + itemsize].view(np_dtype).reshape(-1)
                pts_final[:, i] = values.astype(np.float32)
            except Exception as e:
                self.get_logger().warn(f"解析字段 {ch} 失败: {e}")

        return pts_final

    def pointcloud_callback(self, msg: PointCloud2, sub_key: str) -> None:
        self._last_t = msg.header.stamp

        additional_channels = self.param.subscriber_cfg[sub_key].get("channels", [])
        channels = ["x", "y", "z"] + additional_channels

        try:
            pts_final = self._parse_pointcloud_fields(msg, channels)
            if pts_final is None or pts_final.shape[0] == 0:
                return

            frame_sensor_id = msg.header.frame_id
            transform_sensor_to_map = self.safe_lookup_transform(
                self.map_frame,
                frame_sensor_id,
                msg.header.stamp,
            )
            t = transform_sensor_to_map.transform.translation
            q = transform_sensor_to_map.transform.rotation

            t_np = np.array([t.x, t.y, t.z], dtype=np.float32)
            R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].astype(np.float32)

            self._map.input_pointcloud(
                pts_final,
                channels,
                R,
                t_np,
                0.001,
                0.001,
            )
            self._pointcloud_process_counter += 1

        except Exception as e:
            self.get_logger().error(f"Failed to process point cloud: {e}", exc_info=True)
            return

    def pose_update(self) -> None:
        if self._last_t is None:
            return

        try:
            transform = self.safe_lookup_transform(
                self.map_frame,
                self.base_frame,
                self._last_t,
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed in pose_update: {e}")
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        trans = np.array([t.x, t.y, t.z], dtype=np.float32)
        rot = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3].astype(np.float32)

        self._map.move_to(trans, rot)
        self._map_t = t
        self._map_q = q

    def update_variance(self) -> None:
        self._map.update_variance()

    def update_time(self) -> None:
        self._map.update_time()

    def yaw_from_quaternion(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def extract_layer_in_body_frame(self, layer_name: str, local_length: float):
        if self._map_q is None or self._map_t is None or self._last_t is None:
            return None

        self._map.get_map_with_name_ref(layer_name, self._map_data)

        try:
            transform = self.safe_lookup_transform(
                self.map_frame,
                self.base_frame,
                self._last_t,
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed in extract_layer_in_body_frame: {e}")
            return None

        t = transform.transform.translation
        q = transform.transform.rotation

        # body 原点在 map 系中的位置
        t_map_body = np.array([t.x, t.y, t.z], dtype=np.float32)

        # 只保留 yaw，构造二维旋转
        yaw = self.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        c = np.cos(yaw)
        s = np.sin(yaw)
        R_map_body = np.array(
            [
                [c, -s, 0.0],
                [s,  c, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

        resolution = self._map.resolution
        local_cell_n = int(local_length / resolution)
        if local_cell_n <= 0:
            return None

        body_map = np.full((local_cell_n, local_cell_n), np.nan, dtype=np.float32)

        # 当前 map 数据在 map 系中的覆盖范围
        actual_map_length = (self._map.cell_n - 2) * resolution
        half_map = actual_map_length / 2.0
        origin_x = self._map_t.x - half_map
        origin_y = self._map_t.y - half_map

        # body 局部图范围
        half_local = local_length / 2.0

        for row in range(local_cell_n):
            for col in range(local_cell_n):
                x_body = -half_local + (col + 0.5) * resolution
                y_body = -half_local + (row + 0.5) * resolution

                p_body = np.array([x_body, y_body, 0.0], dtype=np.float32)
                p_map = R_map_body @ p_body + t_map_body

                x_map = p_map[0]
                y_map = p_map[1]

                map_col = int((x_map - origin_x) / resolution)
                map_row = int((y_map - origin_y) / resolution)

                if 0 <= map_row < self._map_data.shape[0] and 0 <= map_col < self._map_data.shape[1]:
                    z_map = self._map_data[map_row, map_col]
                    body_map[row, col] = z_map - t_map_body[2]

        return body_map

    def publish_map_in_body(self, layer_name: str = "elevation", local_length: float = 4.0) -> None:
        body_map = self.extract_layer_in_body_frame(layer_name, local_length)
        if body_map is None:
            return

        gm = GridMap()
        gm.header.frame_id = self.base_frame
        gm.header.stamp = self._last_t if self._last_t is not None else self.get_clock().now().to_msg()

        gm.info.resolution = self._map.resolution
        gm.info.length_x = local_length
        gm.info.length_y = local_length

        gm.info.pose.position.x = 0.0
        gm.info.pose.position.y = 0.0
        gm.info.pose.position.z = 0.0
        gm.info.pose.orientation.w = 1.0
        gm.info.pose.orientation.x = 0.0
        gm.info.pose.orientation.y = 0.0
        gm.info.pose.orientation.z = 0.0

        gm.layers = [layer_name]
        gm.basic_layers = [layer_name]

        arr = Float32MultiArray()
        arr.layout = MAL()
        arr.layout.dim.append(
            MAD(
                label="column_index",
                size=body_map.shape[1],
                stride=body_map.shape[0] * body_map.shape[1],
            )
        )
        arr.layout.dim.append(
            MAD(
                label="row_index",
                size=body_map.shape[0],
                stride=body_map.shape[0],
            )
        )

        arr.data = body_map.flatten().tolist()
        gm.data.append(arr)

        gm.outer_start_index = 0
        gm.inner_start_index = 0

        self._body_map_pub.publish(gm)

    def destroy_node(self) -> None:
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ElevationMappingNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()