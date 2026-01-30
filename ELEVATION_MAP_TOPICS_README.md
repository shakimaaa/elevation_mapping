# Elevation Mapping 话题说明

> 本文档说明 `elevation_mapping_cupy` 节点发布的三个主要高程图话题：结构、层含义、数据流与使用场景。

---

## 一、总览对比

| 项目 | elevation_map_raw | elevation_map_recordable | filtered_elevation_map |
|------|-------------------|---------------------------|------------------------|
| **话题** | `/elevation_mapping/elevation_map_raw` | `/elevation_mapping/elevation_map_recordable` | `/elevation_mapping/filtered_elevation_map` |
| **消息类型** | `grid_map_msgs/msg/GridMap` | `grid_map_msgs/msg/GridMap` | `grid_map_msgs/msg/GridMap` |
| **发布节点** | elevation_mapping_cupy（节点名 `elevation_mapping`） | 同上 | 同上 |
| **数据来源** | 点云直接融合的原始图 | 同 raw，仅发布配置不同 | raw 经 min_filter → smooth → inpaint 后的结果 |
| **典型层** | elevation, traversability, variance, upper_bound | elevation, traversability, variance | inpaint, smooth, min_filter, upper_bound |
| **典型 fps** | 5 Hz | 2 Hz | 5 Hz |
| **主要用途** | 实时算法、平面分割、调试 | 录 bag、离线分析 | 导航、路径规划、代价地图 |

---

## 二、公共消息结构（三者相同）

- **header**：`stamp`（时间戳）、`frame_id`（一般为 `camera_init`，LIO 世界系）
- **info**：
  - `resolution`: 0.04 m
  - `length_x` / `length_y`: 8.0 m
  - `pose`: 地图中心在 `camera_init` 下的位姿（随机器人移动）
- **layout**：200×200 栅格，行优先（row_index stride 200，column_index stride 40000）
- **layers**：本消息包含的层名列表
- **basic_layers**：用于“有效格”判定的基础层（如 RViz 显示）
- **data**：与 `layers` 顺序一致的多层栅格数据，每层 40,000 个 float

---

## 三、各话题详解

### 1. elevation_map_raw（原始高程图）

- **含义**：点云贝叶斯融合后的**未滤波**栅格图，是 elevation_mapping 的“第一手”输出。
- **层**（以当前配置为准）：
  - **elevation**：高程估计（米）；`.nan` = 未观测，有效值约 -0.3～0.4 m
  - **traversability**：可通行性（0–1）；多为 `.nan` 或依赖 elevation
  - **variance**：高程不确定性；`1000.0` = 无效/未观测，小值（如 0.001～0.05）= 有效
  - **upper_bound**：上表面高度，用于障碍/厚度
- **特点**：含不确定性（variance）、可通行性、上界，适合需要“原始+置信度”的下游（平面分割、可通行性分析、调试融合效果）。
- **注意**：大量 `.nan` 和 variance=1000.0 表示未观测，使用前需按层和 variance 做有效性过滤。

---

### 2. elevation_map_recordable（可录制版）

- **含义**：与 raw **同一张图**，只是**发布配置**不同：更少层、更低频率，面向录制与回放。
- **层**（以当前配置为准）：
  - elevation, traversability, variance（**无** upper_bound）
- **特点**：
  - 发布频率通常 2 Hz（raw 为 5 Hz）
  - 层更少 → bag 体积更小
  - 名字即表示：适合 `ros2 bag record` 的“精简版”高程图
- **何时用**：录 bag、离线回放、离线分析；不追求实时、不依赖 upper_bound 时用 recordable 即可。

---

### 3. filtered_elevation_map（滤波后高程图）

- **含义**：对 raw 的 elevation 做**插件流水线**后的结果，输出更平滑、连续、适合规划的地形。
- **层**（以当前配置为准）：
  - **min_filter**：对 elevation 做最小值滤波，抑制尖峰、小障碍
  - **smooth**：在 min_filter 上平滑，降噪
  - **inpaint**：修补空洞（原 `.nan`），得到连续高程场；**basic_layers** 通常用此层
  - **upper_bound**：与 raw 一致，上表面高度
- **数据流**：`raw (elevation) → min_filter → smooth → inpaint`，upper_bound 一并保留并发布。
- **特点**：有效格更多、空洞更少、地形更连续，适合导航、代价地图、路径规划、平面分割等需要“干净地面”的模块。

---

## 四、数据流与配置关系

```
点云 (/cloud_registered_body 等)
    ↓
贝叶斯融合 + 可通行性等
    ↓
内部高程图 (elevation, variance, traversability, upper_bound, …)
    ├─ 按配置发布 → elevation_map_raw        (5 Hz, 4 层)
    ├─ 按配置发布 → elevation_map_recordable (2 Hz, 3 层)
    └─ 经 min_filter → smooth → inpaint (+ upper_bound)
           ↓
       filtered_elevation_map (5 Hz, 4 层)
```

各话题的 `layers`、`basic_layers`、`fps` 在 `publishers` 配置中指定；filtered 的层名由插件流水线（如 plugin_config）决定。

---

## 五、使用建议（何时用哪个）

- **做实时算法、平面分割、要 variance/可通行性/上界**  
  → 用 **elevation_map_raw**。

- **录 bag、离线分析、不关心 upper_bound、希望 bag 小**  
  → 用 **elevation_map_recordable**。

- **做导航、路径规划、代价地图、需要连续无洞的地形**  
  → 用 **filtered_elevation_map**（通常以 **inpaint** 层作为高程）。

- **调试融合/滤波效果**  
  → 同时看 raw（含 variance）和 filtered（inpaint/smooth/min_filter）对比。

---

## 六、数值与有效性约定（三者通用）

- **elevation / inpaint 等高程层**：`.nan` = 未观测或无效；有效值为米制高度（典型场景约 -0.3～0.6 m）。
- **variance**：`1000.0` = 无效/未观测；小数值表示该格估计可靠。
- **traversability**：0–1 或 `.nan`；`.nan` 表示未计算或无效。
- **坐标系**：`frame_id: camera_init`，与 FAST-LIO 等 LIO 世界系一致；`info.pose` 为当前 8×8 m 地图中心在该系下的位姿。

---

## 七、相关配置与文档

- **发布配置**：`elevation_mapping_cupy/config/setups/anymal/anymal_sensor_parameter.yaml` 中的 `publishers` 段。
- **插件流水线**：`plugin_config_file` 指向的 YAML（如 `plugin_config.yaml`）决定 min_filter、smooth、inpaint 等。
- **技术细节**：见同目录下 `ELEVATION_MAPPING_TECHNICAL_GUIDE.md`。
