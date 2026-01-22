# Elevation Mapping CuPy 技术文档

> 完整的高程映射系统技术文档：从点云接收到地图发布的完整流程、函数详解、参数说明与优化建议

---

## 目录

1. [系统概述](#系统概述)
2. [完整数据流程图](#完整数据流程图)
3. [详细函数解析](#详细函数解析)
4. [参数配置说明](#参数配置说明)
5. [优化建议](#优化建议)
6. [关键数据结构](#关键数据结构)

---

## 系统概述

### 系统架构

Elevation Mapping CuPy 是一个基于 GPU 加速的高程地图构建系统，用于实时处理点云数据并生成可用于导航的高程地图。

**核心特性**:
- GPU 加速处理（使用 CuPy 和 CUDA kernels）
- 贝叶斯融合算法
- 实时漂移补偿
- 可见性清理（Ray Tracing）
- 可通行性分析（神经网络）

### 高程地图的 7 个层

| 索引 | 层名称 | 说明 |
|------|--------|------|
| 0 | `elevation` | 高程值（米） |
| 1 | `variance` | 方差（不确定性） |
| 2 | `is_valid` | 有效性标志（0/1） |
| 3 | `traversability` | 可通行性（0-1） |
| 4 | `time` | 时间层（未更新时间） |
| 5 | `upper_bound` | 上界高程 |
| 6 | `is_upper_bound` | 上界标志 |

---

## 完整数据流程图

```
┌─────────────────────────────────────────────────────────────────┐
│                   点云接收到地图发布完整流程                      │
└─────────────────────────────────────────────────────────────────┘

1. ROS 2 点云消息接收
   │
   ├─ 话题订阅: /cloud_registered_body (FAST-LIO)
   └─ 消息类型: sensor_msgs/msg/PointCloud2
   ↓
2. pointcloud_callback() - 点云解析
   │
   ├─ 解析点云字段 (x, y, z, intensity, ...)
   ├─ 获取 TF 变换 (传感器 → 地图坐标系)
   └─ 转换为 NumPy 数组
   ↓
3. input_pointcloud() - 数据预处理
   │
   ├─ 转换为 CuPy 数组 (CPU → GPU)
   ├─ 过滤 NaN 值
   └─ 提取额外通道
   ↓
4. update_map_with_kernel() - 核心更新流程
   │
   ├─ error_counting_kernel() - 误差统计
   │   └─ 计算点云与地图的误差（用于漂移补偿）
   │
   ├─ 漂移补偿判断
   │   └─ 根据误差和噪声阈值决定是否补偿
   │
   ├─ add_points_kernel() - 点云融合 (GPU 并行)
   │   ├─ 坐标变换 (传感器坐标系 → 地图坐标系)
   │   ├─ 网格索引计算
   │   ├─ 异常值检测 (马氏距离)
   │   ├─ 贝叶斯融合更新
   │   │   ├─ 新高程 = (旧高程×新方差 + 新观测×旧方差) / (旧方差+新方差)
   │   │   └─ 新方差 = (旧方差×新方差) / (旧方差+新方差)
   │   └─ 可见性清理 (Ray Tracing)
   │       └─ 清理被遮挡的单元格
   │
   ├─ average_map_kernel() - 地图融合
   │   └─ 将 new_map 的累积值融合到 elevation_map
   │
   ├─ semantic_map.update_layers_pointcloud() - 语义更新
   │   └─ 更新语义层（颜色、类别等）
   │
   ├─ clear_overlap_map() - 重叠清除
   │   └─ 清除重叠区域（多楼层场景）
   │
   ├─ dilation_filter_kernel() - 膨胀滤波
   │   └─ 预处理可通行性计算
   │
   ├─ traversability_filter() - 可通行性计算
   │   └─ 神经网络预测地形可通行性
   │
   └─ update_normal() - 法向量更新
       └─ 更新表面法向量地图
   ↓
5. publish_map() - 地图发布
   │
   ├─ 从 GPU 读取地图数据
   ├─ 转换为 GridMap 消息
   └─ 发布到 ROS 2 话题
   ↓
6. GridMap 消息输出
   │
   └─ 话题: /elevation_mapping_node/elevation_map_raw
```

---

## 详细函数解析

### 阶段 1: 点云接收与解析

#### 函数: `pointcloud_callback()`

**位置**: `elevation_mapping_node.py:361-418`

**功能**: 接收 ROS 2 PointCloud2 消息并解析为 NumPy 数组

**输入参数**:
- `msg: PointCloud2` - ROS 2 点云消息
- `sub_key: str` - 订阅者配置键（如 "fastlio_lidar"）

**关键变量**:
```python
# 从配置获取额外通道
additional_channels = self.param.subscriber_cfg[sub_key].get("channels", [])
# 完整通道列表：["x", "y", "z", ...]
channels = ["x", "y", "z"] + additional_channels

# 点云数据解析
raw_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
points = np.zeros(raw_data.shape[0], dtype=[(ch, np.float32) for ch in channels])

# 字段偏移量映射
field_offsets = {f.name: f.offset for f in msg.fields}

# TF 变换获取
transform_sensor_to_map = self.safe_lookup_transform(
    self.map_frame,      # 目标坐标系（如 "camera_init"）
    frame_sensor_id,     # 源坐标系（如 "body"）
    msg.header.stamp      # 时间戳
)

# 旋转矩阵和平移向量
R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]  # 3x3 旋转矩阵
t_np = np.array([t.x, t.y, t.z], dtype=np.float32)   # 3x1 平移向量

# 准备最终点云矩阵 (N × D)
pts_final = np.zeros((points.shape[0], len(channels)), dtype=np.float32)
for i, ch in enumerate(channels):
    pts_final[:, i] = points[ch]
```

**YAML 配置**:
```yaml
subscribers:
  fastlio_lidar:
    topic_name: "/cloud_registered_body"  # 点云话题
    data_type: pointcloud                  # 数据类型
    channels: ["intensity"]                # 额外通道（可选）
```

**优化建议**:
- 当前固定假设所有字段为 float32，应使用 `PDC_DATATYPE` 字典支持多种数据类型
- 点云解析可优化为批量处理，减少循环开销

---

### 阶段 2: 数据预处理

#### 函数: `input_pointcloud()`

**位置**: `elevation_mapping.py:447-485`

**功能**: 将点云数据转换为 CuPy 数组并过滤无效点

**输入参数**:
- `raw_points: np.ndarray` - 原始点云 (N × D)
- `channels: List[str]` - 通道名称列表
- `R: np.ndarray` - 旋转矩阵 (3×3)
- `t: np.ndarray` - 平移向量 (3×1)
- `position_noise: float` - 位置噪声（当前固定 0.001）
- `orientation_noise: float` - 姿态噪声（当前固定 0.001）

**关键变量**:
```python
# 转换为 CuPy 数组（GPU 内存）
raw_points = cp.asarray(raw_points, dtype=self.data_type)  # 默认 float32

# 提取额外通道（x, y, z 之后的通道）
additional_channels = channels[3:]  # 如 ["intensity"]

# 过滤 NaN 值
raw_points = raw_points[~cp.isnan(raw_points[:, :3]).any(axis=1)]
```

**优化建议**:
- `position_noise` 和 `orientation_noise` 当前为固定值，应从 TF 协方差或配置中获取
- 可添加点云下采样（voxel filter）以减少计算量

---

### 阶段 3: 核心地图更新

#### 函数: `update_map_with_kernel()`

**位置**: `elevation_mapping.py:320-404`

**功能**: 使用 GPU kernel 更新高程地图

**输入参数**:
- `points_all: cp.ndarray` - 完整点云数据 (N × D)
- `channels: List[str]` - 通道列表
- `R: cp.ndarray` - 旋转矩阵 (3×3)
- `t: cp.ndarray` - 平移向量 (3×1)
- `position_noise: float` - 位置噪声
- `orientation_noise: float` - 姿态噪声

**关键变量**:
```python
# 临时缓冲区（清零）
self.new_map *= 0.0  # 存储本次更新的累积值

# 误差统计
error = cp.array([0.0], dtype=cp.float32)      # 误差累加
error_cnt = cp.array([0], dtype=cp.float32)    # 误差计数

# 提取位置信息
points = points_all[:, :3]  # 只取 x, y, z
```

**子函数调用链**:

##### 3.1 误差统计 Kernel

**函数**: `error_counting_kernel()`

**位置**: `custom_kernels.py:286-352`

**功能**: 计算点云与地图的误差，用于漂移补偿

**输入参数**:
- `map` - 当前高程地图
- `p` - 点云数据
- `center_x, center_y` - 地图中心
- `R, t` - 变换矩阵

**输出**:
- `error` - 误差累加值
- `error_cnt` - 有效点数

##### 3.2 漂移补偿

**代码**: `elevation_mapping.py:354-365`

**功能**: 根据误差统计结果补偿累积漂移

**判断条件**:
```python
if (
    enable_drift_compensation                    # 启用漂移补偿
    and error_cnt > min_height_drift_cnt        # 有效点数足够
    and (
        position_noise > position_noise_thresh   # 位置变化大
        or orientation_noise > orientation_noise_thresh  # 姿态变化大
    )
):
    mean_error = error / error_cnt
    if abs(mean_error) < max_drift:              # 误差在允许范围内
        elevation_map[0] += mean_error * drift_compensation_alpha
```

##### 3.3 点云融合 Kernel

**函数**: `add_points_kernel()`

**位置**: `custom_kernels.py:131-283`

**功能**: 将点云融合到地图中（GPU 并行处理）

**输入参数**:
- `center_x, center_y` - 地图中心坐标
- `R` - 旋转矩阵 (3×3)
- `t` - 平移向量 (3×1)
- `norm_map` - 法向量地图
- `p` - 点云数据 (N×3)
- `map` - 高程地图（7层）
- `newmap` - 新地图缓冲区

**处理流程**:
1. **坐标变换**: 传感器坐标系 → 地图坐标系
   ```cuda
   U x = transform_p(rx, ry, rz, R[0], R[1], R[2], t[0]);
   U y = transform_p(rx, ry, rz, R[3], R[4], R[5], t[1]);
   U z = transform_p(rx, ry, rz, R[6], R[7], R[8], t[2]);
   ```

2. **网格索引计算**
   ```cuda
   int idx = get_idx(x, y, center_x[0], center_y[0]);
   ```

3. **异常值检测**（马氏距离）
   ```cuda
   if (abs(map_h - z) > (map_v * mahalanobis_thresh)) {
       // 异常值，增加方差
       atomicAdd(&map[get_map_idx(idx, 1)], outlier_variance);
   }
   ```

4. **贝叶斯融合更新**
   ```cuda
   T new_h = (map_h * v + z * map_v) / (map_v + v);
   T new_v = (map_v * v) / (map_v + v);
   atomicAdd(&newmap[get_map_idx(idx, 0)], new_h);
   atomicAdd(&newmap[get_map_idx(idx, 1)], new_v);
   atomicAdd(&newmap[get_map_idx(idx, 2)], 1.0);
   ```

5. **可见性清理**（Ray Tracing）
   ```cuda
   // 沿着从传感器到点的射线遍历
   for (float16 s = ray_step; s < ray_length; s += ray_step) {
       // 如果射线穿透了某个单元格，降低其有效性
       if (nmap_h > nz + 0.01) {
           atomicAdd(&map[get_map_idx(nidx, 2)], -cleanup_step);
       }
   }
   ```

##### 3.4 地图融合 Kernel

**函数**: `average_map_kernel()`

**位置**: `custom_kernels.py:354-395`

**功能**: 将 `new_map` 的累积值融合到 `elevation_map`

**处理逻辑**:
```cuda
if (new_cnt > 0) {  // 有新点更新
    if (new_v / new_cnt > max_variance) {
        // 方差过大，重置
        map[高程] = 0;
        map[方差] = initial_variance;
        map[有效性] = 0;
    } else {
        // 平均融合
        map[高程] = new_h / new_cnt;
        map[方差] = new_v / new_cnt;
        map[有效性] = 1;
    }
}
```

##### 3.5 语义地图更新

**函数**: `semantic_map.update_layers_pointcloud()`

**位置**: `semantic_map.py:223-260`

**功能**: 更新语义层（如颜色、类别等）

**融合算法**（根据配置选择）:
- `average` - 平均值融合
- `bayesian_inference` - 贝叶斯推理
- `class_average` - 类别平均值
- `color` - 颜色融合

##### 3.6 重叠清除

**函数**: `clear_overlap_map()`

**位置**: `elevation_mapping.py:406-435`

**功能**: 清除重叠区域（用于多楼层场景）

##### 3.7 可通行性计算

**函数**: `dilation_filter_kernel()` + `traversability_filter()`

**位置**: `custom_kernels.py:398-...` + `elevation_mapping.py:398`

**功能**: 计算地形可通行性

**处理流程**:
1. 膨胀滤波预处理
2. 神经网络可通行性预测（Chainer/PyTorch）

---

### 阶段 4: 地图发布

#### 函数: `publish_map()`

**位置**: `elevation_mapping_node.py:281-316`

**功能**: 将高程地图转换为 GridMap 消息并发布

**输入参数**:
- `key: str` - 发布者配置键

**关键变量**:
```python
# GridMap 消息构建
gm = GridMap()
gm.header.frame_id = self.map_frame          # 坐标系
gm.info.resolution = self._map.resolution    # 分辨率
gm.info.length_x = actual_map_length         # 地图长度
gm.info.length_y = actual_map_length
gm.info.pose.position.x = self._map_t.x      # 地图中心位置
gm.info.pose.position.y = self._map_t.y

# 发布配置的层
for layer in self.my_publishers[key].get("layers", []):
    self._map.get_map_with_name_ref(layer, self._map_data)
    # 转换为 Float32MultiArray
    arr.data = map_data_for_gridmap.flatten().tolist()
    gm.data.append(arr)
```

---

## 参数配置说明

### YAML 配置文件结构

```yaml
elevation_mapping:
  ros__parameters:
    #### 基础参数 ########
    resolution: 0.1                    # 地图分辨率（米）
    map_length: 20.0                   # 地图大小（米）
    
    #### 订阅者配置 ########
    subscribers:
      fastlio_lidar:
        topic_name: "/cloud_registered_body"
        data_type: pointcloud
        channels: ["intensity"]         # 可选额外通道
    
    #### 发布者配置 ########
    publishers:
      elevation_map_raw:
        layers: ['elevation', 'traversability', 'variance', 'upper_bound']
        basic_layers: ['elevation', 'traversability']
        fps: 5.0
    
    #### 传感器参数 ########
    sensor_noise_factor: 0.05          # 传感器噪声因子
    min_valid_distance: 0.1            # 最小有效距离
    max_height_range: 10.5             # 最大高度范围
    
    #### 融合参数 ########
    mahalanobis_thresh: 2.0            # 马氏距离阈值
    outlier_variance: 0.01             # 异常值方差
    
    #### 漂移补偿 ########
    enable_drift_compensation: true
    max_drift: 0.1
    drift_compensation_alpha: 0.1
    min_height_drift_cnt: 100
    
    #### 可见性清理 ########
    enable_visibility_cleanup: true
    max_ray_length: 10.0
    cleanup_step: 0.1
    cleanup_cos_thresh: 0.1
    
    #### 可通行性 ########
    dilation_size: 3
    use_chainer: false                 # false 使用 PyTorch
    weight_file: 'config/weights.dat'
```

### 关键参数详解

| 参数类别 | 参数名 | 默认值 | 说明 | 优化方向 |
|---------|--------|--------|------|---------|
| **基础参数** | `resolution` | 0.04 | 地图分辨率（米） | 性能↗精度↘ |
| | `map_length` | 8.0 | 地图大小（米） | 性能↗精度↘ |
| **传感器参数** | `sensor_noise_factor` | 0.05 | 传感器噪声因子 | 根据传感器特性调整 |
| | `min_valid_distance` | 0.3 | 最小有效距离 | 过滤近距离噪声 |
| | `max_height_range` | 1.0 | 最大高度范围 | 过滤天花板 |
| **融合参数** | `mahalanobis_thresh` | 2.0 | 马氏距离阈值 | 异常值检测敏感度 |
| | `outlier_variance` | 0.01 | 异常值方差 | 异常值惩罚程度 |
| **漂移补偿** | `enable_drift_compensation` | true | 启用漂移补偿 | 精度优化 |
| | `max_drift` | 0.1 | 最大漂移 | 安全性参数 |
| **可见性清理** | `enable_visibility_cleanup` | true | 启用可见性清理 | 性能优化 |
| | `max_ray_length` | 2.0 | 最大射线长度 | 计算量↗精度↗ |
| **可通行性** | `dilation_size` | 2 | 膨胀滤波大小 | 平滑度调整 |
| | `use_chainer` | true | 使用 Chainer | PyTorch 更快但更占内存 |

---

## 优化建议

### 1. 点云解析优化

**当前问题**:
- 固定假设所有字段为 float32
- 逐字段解析效率低

**优化方案**:
```python
# 使用 PDC_DATATYPE 字典支持多种数据类型
field_info = {}
for field in msg.fields:
    datatype_str = str(field.datatype)
    dtype = PDC_DATATYPE.get(datatype_str, np.float32)
    field_info[field.name] = {
        'offset': field.offset,
        'dtype': dtype,
        'size': get_field_size(datatype_str)
    }

# 批量解析
points = parse_pointcloud_batch(raw_data, field_info, channels)
```

### 2. 点云下采样

**优化方案**:
```python
# 在 input_pointcloud() 中添加 voxel filter
if self.param.voxel_filter_size > 0:
    raw_points = self.voxel_filter(raw_points, self.param.voxel_filter_size)
```

**YAML 参数**:
```yaml
voxel_filter_size: 0.1  # 体素大小（米），0 表示不滤波
```

### 3. 噪声参数优化

**当前问题**:
- `position_noise` 和 `orientation_noise` 为固定值

**优化方案**:
```python
# 从 TF 协方差获取
transform = self.safe_lookup_transform(...)
if hasattr(transform, 'covariance'):
    position_noise = calculate_position_noise(transform.covariance)
    orientation_noise = calculate_orientation_noise(transform.covariance)
else:
    # 使用配置的默认值
    position_noise = self.param.default_position_noise
    orientation_noise = self.param.default_orientation_noise
```

### 4. GPU 内存优化

**优化方案**:
- 使用 CuPy 内存池减少分配开销
- 复用缓冲区，减少内存分配
- 异步传输，重叠计算与传输

```python
# 使用内存池
cp.cuda.set_allocator(cp.cuda.MemoryPool().malloc)

# 复用缓冲区
if not hasattr(self, '_points_buffer'):
    self._points_buffer = cp.zeros((max_points, max_channels), dtype=cp.float32)
```

### 5. 并行处理优化

**优化方案**:
- 使用多线程处理多个订阅者
- 异步处理点云和图像数据

```python
# 使用线程池
from concurrent.futures import ThreadPoolExecutor
executor = ThreadPoolExecutor(max_workers=4)
```

### 6. 参数调优建议

**性能优化参数**:
```yaml
# 降低分辨率（如果精度要求不高）
resolution: 0.1  # 从 0.04 增加到 0.1，减少 6.25 倍计算量

# 减少地图大小
map_length: 20.0  # 根据实际需求调整

# 禁用不必要的功能
enable_visibility_cleanup: false  # 如果不需要，可以禁用
enable_overlap_clearance: false   # 单层环境可以禁用

# 调整发布频率
fps: 5.0  # 降低发布频率，减少 CPU 开销
```

**精度优化参数**:
```yaml
# 提高分辨率
resolution: 0.04  # 更高精度

# 增加地图大小
map_length: 30.0  # 更大的覆盖范围

# 启用所有功能
enable_visibility_cleanup: true
enable_overlap_clearance: true
enable_drift_compensation: true
```

---

## 关键数据结构

### 高程地图结构

```python
# elevation_map: (7, cell_n, cell_n) CuPy 数组
elevation_map[0]  # elevation: 高程值（米）
elevation_map[1]  # variance: 方差（不确定性）
elevation_map[2]  # is_valid: 有效性标志（0/1）
elevation_map[3]  # traversability: 可通行性（0-1）
elevation_map[4]  # time: 时间层（未更新时间）
elevation_map[5]  # upper_bound: 上界高程
elevation_map[6]  # is_upper_bound: 上界标志
```

### 点云数据结构

```python
# raw_points: (N, D) NumPy/CuPy 数组
# N: 点数
# D: 维度数（至少 3，包含 x, y, z，可能还有 intensity, rgb 等）

# 示例：FAST-LIO 点云
points = np.array([
    [x1, y1, z1, intensity1],  # 点 1
    [x2, y2, z2, intensity2],  # 点 2
    ...
], dtype=np.float32)

channels = ["x", "y", "z", "intensity"]
```

### 变换矩阵

```python
# R: 旋转矩阵 (3×3)
R = np.array([
    [r00, r01, r02],
    [r10, r11, r12],
    [r20, r21, r22]
], dtype=np.float32)

# t: 平移向量 (3×1)
t = np.array([tx, ty, tz], dtype=np.float32)
```

---

## 总结

本文档详细介绍了 Elevation Mapping CuPy 系统的完整工作流程，包括：

1. **数据流**: 从点云接收到地图发布的完整流程
2. **函数详解**: 每个关键函数的输入输出、处理逻辑
3. **参数配置**: YAML 配置参数的详细说明
4. **优化建议**: 性能优化和精度优化的具体方案

通过理解这些内容，您可以：
- 更好地配置系统参数
- 针对特定场景进行优化
- 调试和解决性能问题
- 扩展系统功能

---

## 参考

- 源码位置: `src/elevation/elevation_mapping/elevation_mapping_cupy/`
- 配置文件: `config/core/core_param.yaml`
- 环境配置: `ELEVATION_MAPPING_ENV.md`
