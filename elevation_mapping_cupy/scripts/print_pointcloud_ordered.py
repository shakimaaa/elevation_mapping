#!/usr/bin/env python3
"""订阅 sensor_msgs/PointCloud2，按消息内二进制顺序解码并打印 (索引, x, y, z)。

anymal.launch.py 里节点名是 elevation_mapping，点云话题为:
  /elevation_mapping/body_elevation_cloud
（不是 elevation_mapping_node；节点构造名会被 launch 的 name= 覆盖。）

用法示例:
  ros2 run elevation_mapping_cupy print_pointcloud_ordered.py

  ros2 run elevation_mapping_cupy print_pointcloud_ordered.py \\
    --ros-args -p topic:=/elevation_mapping/body_elevation_cloud -p max_points:=40
"""
from __future__ import annotations

import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from sensor_msgs.msg import PointCloud2, PointField


def _fmt_for_field(f: PointField) -> tuple[str, int]:
    if f.datatype == PointField.FLOAT32:
        return "<f", 4
    if f.datatype == PointField.FLOAT64:
        return "<d", 8
    raise ValueError(f"不支持字段 {f.name} 的 datatype={f.datatype}，请用 x/y/z float32")


def decode_xyz_ordered(msg: PointCloud2) -> list[tuple[float, float, float]]:
    """按 point_step 逐点顺序解码，与 ROS 消息布局一致。"""
    fields = {f.name: f for f in msg.fields}
    for name in ("x", "y", "z"):
        if name not in fields:
            raise ValueError(f"消息缺字段 {name}，当前: {list(fields)}")

    ox = fields["x"].offset
    oy = fields["y"].offset
    oz = fields["z"].offset
    fmt_x, _ = _fmt_for_field(fields["x"])
    fmt_y, _ = _fmt_for_field(fields["y"])
    fmt_z, _ = _fmt_for_field(fields["z"])

    step = msg.point_step
    if step <= 0 or len(msg.data) < step:
        return []

    n = int(msg.width) * int(msg.height)
    out: list[tuple[float, float, float]] = []
    raw = memoryview(msg.data)
    for i in range(n):
        base = i * step
        if base + step > len(raw):
            break
        x = struct.unpack_from(fmt_x, raw, base + ox)[0]
        y = struct.unpack_from(fmt_y, raw, base + oy)[0]
        z = struct.unpack_from(fmt_z, raw, base + oz)[0]
        out.append((float(x), float(y), float(z)))
    return out


class PointCloudOrderedPrinter(Node):
    def __init__(self) -> None:
        super().__init__("print_pointcloud_ordered")
        self.declare_parameter("topic", "/elevation_mapping/body_elevation_cloud")
        self.declare_parameter("max_points", 0)
        topic = self.get_parameter("topic").get_parameter_value().string_value
        self._max = int(self.get_parameter("max_points").get_parameter_value().integer_value)

        # 与 rclpy 默认 Publisher（RELIABLE + VOLATILE）一致，避免 BEST_EFFORT 订阅收不到
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, topic, self._cb, qos)
        self.get_logger().info(f"订阅 {topic}，按接收消息内顺序打印点；max_points=0 表示全部")

    def _cb(self, msg: PointCloud2) -> None:
        try:
            pts = decode_xyz_ordered(msg)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        n = len(pts)
        limit = n if self._max <= 0 else min(n, self._max)
        self.get_logger().info(
            f"--- frame={msg.header.frame_id} width={msg.width} height={msg.height} "
            f"points={n} point_step={msg.point_step} 显示前 {limit} 个 ---"
        )
        for i in range(limit):
            x, y, z = pts[i]
            print(f"{i:5d}  {x:10.4f} {y:10.4f} {z:10.4f}")
        if self._max > 0 and n > limit:
            print(f"... 省略 {n - limit} 个点 (max_points={self._max})")
        sys.stdout.flush()


def main() -> None:
    rclpy.init()
    node = PointCloudOrderedPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
