# Elevation Mapping CuPy 环境配置

> 专门针对 elevation_mapping_cupy 包的完整环境配置清单

## 1. 基础环境要求

- **操作系统**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS 发行版**: ROS 2 Humble
- **Python 版本**: Python 3.10+
- **GPU**: NVIDIA GPU with CUDA 支持

## 2. CUDA 运行时库 (必需)

### 安装的包
- `cuda-cudart-12-1` - CUDA 运行时库
- `cuda-nvrtc-12-1` - NVRTC 库（CuPy 必需）
- `cuda-nvrtc-dev-12-1` - NVRTC 开发库

### 安装命令
```bash
# 添加 CUDA 仓库
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb -O /tmp/cuda-keyring.deb
sudo dpkg -i /tmp/cuda-keyring.deb
sudo apt-get update

# 安装 CUDA 运行时库
sudo apt-get install -y --no-install-recommends \
    cuda-cudart-12-1 \
    cuda-nvrtc-12-1 \
    cuda-nvrtc-dev-12-1
```

### 环境变量配置
```bash
# 添加到 ~/.bashrc
export LD_LIBRARY_PATH=/usr/local/cuda-12.1/lib64:/usr/local/cuda/lib64:${LD_LIBRARY_PATH}
export PATH=/usr/local/cuda-12.1/bin:${PATH}

# 更新库缓存
sudo ldconfig
```

## 3. Python 依赖包 (pip 安装)

### GPU 加速库 (必需)
```bash
# PyTorch (CUDA 12.1)
python3 -m pip install -U --extra-index-url https://download.pytorch.org/whl/cu121 \
    torch \
    torchvision \
    torchaudio

# CuPy (CUDA 12.x)
python3 -m pip install cupy-cuda12x
```

**版本信息:**
- `torch`: 2.9.1
- `torchvision`: 0.24.1
- `torchaudio`: 2.9.1
- `cupy-cuda12x`: 13.6.0

### 科学计算库 (必需)
```bash
python3 -m pip install \
    "numpy<2.0.0" \
    scipy \
    scikit-learn
```

**版本信息:**
- `numpy`: 1.24.2 (必须 < 2.0.0)
- `scipy`: 1.15.3
- `scikit-learn`: 1.7.2

### 计算机视觉库 (必需)
```bash
python3 -m pip install opencv-python
```

**版本信息:**
- `opencv-python`: 4.11.0.86

### ROS2 转换工具 (必需)
```bash
python3 -m pip install ros2_numpy
```

**版本信息:**
- `ros2_numpy`: 0.0.5

### 其他工具库 (必需)
```bash
python3 -m pip install \
    simple-parsing \
    transforms3d
```

**版本信息:**
- `simple-parsing`: 0.1.7
- `transforms3d`: 0.4.2

## 4. ROS2 功能包 (apt 安装)

### Grid Map 系列 (核心依赖)
```bash
sudo apt-get install -y --no-install-recommends \
    ros-humble-grid-map-msgs \
    ros-humble-grid-map-ros \
    ros-humble-grid-map-core \
    ros-humble-grid-map-cv \
    ros-humble-grid-map-demos
```

**包列表:**
- `ros-humble-grid-map-msgs` - 消息定义
- `ros-humble-grid-map-ros` - ROS 接口
- `ros-humble-grid-map-core` - 核心功能
- `ros-humble-grid-map-cv` - OpenCV 处理
- `ros-humble-grid-map-demos` - 示例代码

### 图像处理 (必需)
```bash
sudo apt-get install -y --no-install-recommends \
    ros-humble-image-transport \
    ros-humble-cv-bridge
```

**说明:**
- `image-transport`: 优化图像话题传输
- `cv-bridge`: OpenCV 和 ROS 图像消息转换

### 点云处理 (必需)
```bash
sudo apt-get install -y --no-install-recommends \
    ros-humble-pcl-ros \
    ros-humble-point-cloud-transport
```

**说明:**
- `pcl-ros`: PCL 点云库 ROS 接口
- `point-cloud-transport`: 点云压缩传输

### 坐标变换 (必需)
```bash
sudo apt-get install -y --no-install-recommends \
    ros-humble-tf-transformations
```

**说明:**
- 用于坐标变换计算

### 可视化工具 (可选，推荐)
```bash
sudo apt-get install -y --no-install-recommends \
    ros-humble-rviz2
```

**说明:**
- 用于可视化高程地图

## 5. 系统库和开发工具 (apt 安装)

### C++ 开发库
```bash
sudo apt-get install -y --no-install-recommends \
    libboost-all-dev \
    libeigen3-dev \
    pybind11-dev
```

**说明:**
- `libboost-all-dev`: Boost C++ 库
- `libeigen3-dev`: Eigen3 线性代数库
- `pybind11-dev`: C++/Python 绑定工具

### Python 系统包
```bash
sudo apt-get install -y --no-install-recommends \
    python3-shapely \
    python3-ruamel.yaml \
    python3-transforms3d \
    python3-scipy \
    python3-opencv \
    python3-numpy
```

**说明:**
- 系统级的 Python 包，部分可能与 pip 安装的版本共存

## 6. 完整安装脚本

```bash
#!/bin/bash
set -e

echo "=== 安装 Elevation Mapping CuPy 环境 ==="

# 1. 安装 pip (如果未安装)
sudo apt-get update
sudo apt-get install -y python3-pip python3-dev

# 2. 安装 CUDA 运行时库
echo "安装 CUDA 运行时库..."
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb -O /tmp/cuda-keyring.deb
sudo dpkg -i /tmp/cuda-keyring.deb
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    cuda-cudart-12-1 \
    cuda-nvrtc-12-1 \
    cuda-nvrtc-dev-12-1

# 3. 安装 Python GPU 库
echo "安装 PyTorch 和 CuPy..."
python3 -m pip install -U --extra-index-url https://download.pytorch.org/whl/cu121 \
    torch torchvision torchaudio
python3 -m pip install cupy-cuda12x

# 4. 安装 Python 科学计算库
echo "安装 Python 科学计算库..."
python3 -m pip install "numpy<2.0.0" scipy scikit-learn opencv-python

# 5. 安装 ROS2 相关 Python 包
echo "安装 ROS2 Python 包..."
python3 -m pip install ros2_numpy simple-parsing transforms3d

# 6. 安装 ROS2 功能包
echo "安装 ROS2 功能包..."
sudo apt-get install -y --no-install-recommends \
    libboost-all-dev \
    libeigen3-dev \
    pybind11-dev \
    ros-humble-grid-map-msgs \
    ros-humble-grid-map-ros \
    ros-humble-grid-map-core \
    ros-humble-grid-map-cv \
    ros-humble-grid-map-demos \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-pcl-ros \
    ros-humble-point-cloud-transport \
    ros-humble-tf-transformations \
    ros-humble-rviz2 \
    python3-shapely \
    python3-ruamel.yaml \
    python3-transforms3d \
    python3-scipy \
    python3-opencv \
    python3-numpy

# 7. 配置环境变量
echo "配置环境变量..."
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.1/lib64:/usr/local/cuda/lib64:${LD_LIBRARY_PATH}' >> ~/.bashrc
echo 'export PATH=/usr/local/cuda-12.1/bin:${PATH}' >> ~/.bashrc
export LD_LIBRARY_PATH=/usr/local/cuda-12.1/lib64:/usr/local/cuda/lib64:${LD_LIBRARY_PATH}
sudo ldconfig

echo "=== 安装完成 ==="
echo "请运行 'source ~/.bashrc' 或重新打开终端以加载环境变量"
```

## 7. 验证安装

### 验证 CUDA 和 CuPy
```bash
python3 -c "import cupy; print(f'CuPy 版本: {cupy.__version__}'); print(f'CUDA 版本: {cupy.cuda.runtime.runtimeGetVersion()}')"
```

预期输出:
```
CuPy 版本: 13.6.0
CUDA 版本: 12090
```

### 验证 ROS2 包
```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep grid-map
```

### 验证 CUDA 库
```bash
ldconfig -p | grep nvrtc
```

## 8. 关键版本要求

| 组件 | 版本要求 | 说明 |
|------|---------|------|
| NumPy | < 2.0.0 | elevation_mapping_cupy 不兼容 NumPy 2.0+ |
| CUDA | 12.1 | cupy-cuda12x 需要 CUDA 12.x 运行时 |
| PyTorch | 2.9.1 | 支持 CUDA 12.1 |
| CuPy | 13.6.0 | 使用 cupy-cuda12x 包 |

## 9. 常见问题

### libnvrtc.so.12 缺失
**错误**: `OSError: libnvrtc.so.12: cannot open shared object file`

**解决**: 安装 CUDA 运行时库（见第2节）

### NumPy 版本不兼容
**错误**: NumPy 2.0+ 导致的兼容性问题

**解决**: 使用 `numpy<2.0.0`，推荐 1.24.2

### ros2_numpy 未找到
**错误**: `ModuleNotFoundError: No module named 'ros2_numpy'`

**解决**: `pip install ros2_numpy`

## 10. 参考文档

- elevation_mapping_cupy 源码: `src/elevation/elevation_mapping/elevation_mapping_cupy/`
- 参考 Dockerfile: `src/elevation/elevation_mapping/docker/Dockerfile.x64mix`
- 编译脚本: `src/elevation/elevation_mapping/docker/build.sh`
