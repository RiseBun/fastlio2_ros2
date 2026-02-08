# FASTLIO2 ROS2 - 高性能LiDAR-惯性里程计系统

## 目录

- [1. 系统概述](#1-系统概述)
  - [1.1 核心功能](#11-核心功能)
  - [1.2 应用场景](#12-应用场景)
  - [1.3 算法原理架构](#13-算法原理架构)
- [2. 配置详解](#2-配置详解)
  - [2.1 传感器配置](#21-传感器配置)
  - [2.2 滤波与地图参数](#22-滤波与地图参数)
  - [2.3 IMU噪声参数](#23-imu噪声参数)
  - [2.4 关键帧管理](#24-关键帧管理)
  - [2.5 回环检测参数](#25-回环检测参数)
  - [2.6 内存管理参数](#26-内存管理参数)
  - [2.7 重定位参数](#27-重定位参数)
  - [2.8 场景配置建议](#28-场景配置建议)
- [3. 安装与部署](#3-安装与部署)
  - [3.1 系统要求](#31-系统要求)
  - [3.2 依赖安装](#32-依赖安装)
  - [3.3 编译安装](#33-编译安装)
  - [3.4 嵌入式平台配置](#34-嵌入式平台配置)
- [4. 运行指南](#4-运行指南)
  - [4.1 启动命令](#41-启动命令)
  - [4.2 数据流处理](#42-数据流处理)
  - [4.3 关键帧管理机制](#43-关键帧管理机制)
  - [4.4 性能优化策略](#44-性能优化策略)
- [5. GUI界面功能](#5-gui界面功能)
  - [5.1 纯建图标签页](#51-纯建图标签页)
  - [5.2 回环检测标签页](#52-回环检测标签页)
  - [5.3 重定位标签页](#53-重定位标签页)
  - [5.4 高级工具标签页](#54-高级工具标签页)
- [6. 高级功能特性](#6-高级功能特性)
  - [6.1 回环检测机制](#61-回环检测机制)
  - [6.2 重定位功能](#62-重定位功能)
  - [6.3 增量地图保存](#63-增量地图保存)
  - [6.4 Session合并](#64-session合并)
- [7. 故障排除](#7-故障排除)
- [8. 性能评估](#8-性能评估)

---

## 1. 系统概述

### 1.1 核心功能

FASTLIO2 ROS2是一个高性能的紧耦合LiDAR-惯性里程计系统，基于迭代误差状态卡尔曼滤波器(IESKF)实现。本系统在原始FAST-LIO2基础上进行了大量扩展，主要功能包括：

| 功能模块 | 描述 |
|---------|------|
| **实时建图** | 基于IESKF的紧耦合LiDAR-IMU融合SLAM |
| **回环检测** | 三级验证机制：距离搜索 + ScanContext + ICP |
| **位姿图优化** | 基于GTSAM的全局位姿图优化 |
| **重定位** | 在已建地图中自动定位 |
| **增量建图** | 多Session合并，支持大规模地图构建 |
| **内存管理** | 自动清理旧关键帧点云，适配嵌入式平台 |
| **GUI控制面板** | 可视化参数配置和系统监控 |

### 1.2 应用场景

- **自主移动机器人导航**：室内外环境下的实时定位与建图
- **无人机航测**：大范围环境的三维重建
- **自动驾驶**：车辆的实时定位与障碍物感知
- **AR/VR设备定位**：实时位姿估计
- **数字孪生**：环境的精确三维建模

### 1.3 算法原理架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                        FASTLIO2 系统架构                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────┐   ┌──────────┐   ┌───────────────────────────────┐   │
│  │ LiDAR    │──▶│ 预处理    │──▶│ 点云降采样 + 范围滤波          │   │
│  │ /livox   │   │ 模块      │   │ scan_resolution: 0.15m       │   │
│  └──────────┘   └──────────┘   └───────────┬───────────────────┘   │
│                                             │                        │
│  ┌──────────┐   ┌──────────┐               ▼                        │
│  │ IMU      │──▶│ IMU      │──▶┌───────────────────────────────┐   │
│  │ /livox   │   │ 初始化    │   │         IESKF 状态估计         │   │
│  └──────────┘   └──────────┘   │  ┌─────────────────────────┐  │   │
│                                 │  │ 状态: [R, p, v, bg, ba] │  │   │
│                                 │  │ 迭代更新 (max 5次)      │  │   │
│                                 │  └─────────────────────────┘  │   │
│                                 └───────────┬───────────────────┘   │
│                                             │                        │
│                                             ▼                        │
│  ┌───────────────────────────────────────────────────────────────┐ │
│  │                    ikd-Tree 动态地图                           │ │
│  │  ● 增量式KD树结构      ● 支持点云增删                          │ │
│  │  ● 近邻搜索加速        ● 地图自动裁剪 (cube_len: 300m)        │ │
│  └───────────────────────────────────────────────────────────────┘ │
│                                             │                        │
│                                             ▼                        │
│  ┌───────────────────────────────────────────────────────────────┐ │
│  │                    关键帧管理 (KeyFrameManager)                 │ │
│  │  ● 距离阈值: 1.0m      ● 角度阈值: 0.2rad (~11.5°)            │ │
│  │  ● 点云降采样: 0.15m   ● 自动内存清理: 500帧限制              │ │
│  └───────────────────────────────────────────────────────────────┘ │
│                                             │                        │
│                                             ▼                        │
│  ┌───────────────────────────────────────────────────────────────┐ │
│  │                    回环检测 (LoopDetector)                      │ │
│  │                                                                 │ │
│  │  第1级: 距离搜索 ──▶ 第2级: ScanContext ──▶ 第3级: ICP验证    │ │
│  │  radius: 15m         sc_thresh: 0.20       icp_thresh: 0.5     │ │
│  │                                                                 │ │
│  └───────────────────────────────────────────────────────────────┘ │
│                                             │                        │
│                                             ▼                        │
│  ┌───────────────────────────────────────────────────────────────┐ │
│  │                 位姿图优化 (PoseGraphOptimizer)                  │ │
│  │  ● 基于GTSAM 4.2.0    ● 增量式优化                             │ │
│  │  ● 里程计约束 + 回环约束                                        │ │
│  └───────────────────────────────────────────────────────────────┘ │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

**核心算法流程**：

1. **IMU前向传播**：利用IMU数据预测状态，建立运动模型
2. **点云预处理**：时间排序、降采样、范围过滤
3. **迭代更新**：通过ikd-Tree进行近邻搜索，计算点到面残差，迭代优化状态
4. **地图更新**：将处理后的点云加入ikd-Tree
5. **关键帧选择**：基于位移和角度变化判断是否插入新关键帧
6. **回环检测**：独立线程执行三级验证
7. **位姿图优化**：检测到回环后，使用GTSAM优化全局位姿

---

## 2. 配置详解

配置文件位于：`fastlio2/config/lio.yaml`

### 2.1 传感器配置

```yaml
# 传感器话题配置
imu_topic: /livox/imu          # IMU数据话题
lidar_topic: /livox/lidar      # LiDAR点云话题
body_frame: body               # 机体坐标系名称
world_frame: lidar             # 世界坐标系名称
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `imu_topic` | string | /livox/imu | IMU传感器数据发布话题 |
| `lidar_topic` | string | /livox/lidar | LiDAR点云数据话题 |
| `body_frame` | string | body | 用于TF发布的机体坐标系 |
| `world_frame` | string | lidar | 世界/地图坐标系 |

### 2.2 滤波与地图参数

```yaml
# 点云预处理
lidar_filter_num: 6           # 点云抽样率 (每N个点取1个)
lidar_min_range: 0.5          # 最小有效距离 (m)
lidar_max_range: 30.0         # 最大有效距离 (m)
scan_resolution: 0.15         # 扫描点云降采样分辨率 (m)
map_resolution: 0.3           # 地图点云降采样分辨率 (m)

# ikd-Tree地图配置
cube_len: 300                 # 地图立方体边长 (m)
det_range: 60                 # 有效探测范围 (m)
move_thresh: 1.5              # 地图移动阈值 (m)
```

**参数影响分析**：

| 参数 | 增大影响 | 减小影响 | 推荐范围 |
|------|---------|---------|---------|
| `lidar_filter_num` | 减少计算量，降低精度 | 提高精度，增加计算量 | 3-8 |
| `scan_resolution` | 减少点数，加快处理 | 保留更多细节 | 0.1-0.3 |
| `map_resolution` | 减少地图大小 | 提高地图精度 | 0.2-0.5 |
| `cube_len` | 支持更大场景 | 减少内存占用 | 200-500 |
| `det_range` | 扩大匹配范围 | 减少计算量 | 40-100 |

### 2.3 IMU噪声参数

```yaml
# IMU噪声参数 (根据传感器规格调整)
na: 0.01                      # 加速度计噪声密度 (m/s²/√Hz)
ng: 0.01                      # 陀螺仪噪声密度 (rad/s/√Hz)
nba: 0.0001                   # 加速度计偏置随机游走
nbg: 0.0001                   # 陀螺仪偏置随机游走
```

**典型传感器IMU噪声参数参考**：

| 传感器 | na | ng | nba | nbg |
|--------|-----|-----|------|------|
| Livox MID-360 | 0.01 | 0.01 | 0.0001 | 0.0001 |
| Livox Avia | 0.02 | 0.02 | 0.0002 | 0.0002 |
| 低成本MEMS | 0.05 | 0.05 | 0.001 | 0.001 |

### 2.4 关键帧管理

```yaml
# 关键帧选择
keyframe_dist_threshold: 1.0     # 距离阈值 (m)
keyframe_angle_threshold: 0.2    # 角度阈值 (rad, ~11.5°)
```

**选择策略**：当机器人相对上一关键帧的**位移 >= 1.0m** 或 **旋转角度 >= 0.2rad** 时，插入新关键帧。

### 2.5 回环检测参数

```yaml
# 回环检测配置
loop_closure_enable: true         # 启用回环检测
loop_closure_frequency: 1.0       # 检测频率 (Hz)
loop_search_radius: 15.0          # 空间搜索半径 (m)
loop_time_diff_threshold: 30.0    # 最小时间间隔 (s)
sc_dist_threshold: 0.20           # ScanContext相似度阈值 (越小越严格)
icp_fitness_threshold: 0.5        # ICP适配度阈值 (越小越严格)
submap_size: 25                   # 子图包含的关键帧数
```

**三级验证机制详解**：

```
候选帧选择 ──▶ ScanContext验证 ──▶ ICP几何验证 ──▶ 回环确认
   │                │                  │
   │ 距离 < 15m     │ SC距离 < 0.20    │ ICP适配度 < 0.5
   │ 时间 > 30s     │ 提供粗略旋转估计  │ 精确相对位姿
   ▼                ▼                  ▼
 快速筛选        特征描述子匹配     点云几何验证
```

**参数调优建议**：

| 场景 | sc_dist_threshold | icp_fitness_threshold | 说明 |
|------|-------------------|----------------------|------|
| 结构化环境 | 0.15-0.20 | 0.3-0.5 | 办公室、走廊等 |
| 复杂户外 | 0.25-0.35 | 0.5-0.8 | 植被、不规则地形 |
| 高重复场景 | 0.10-0.15 | 0.2-0.3 | 避免误匹配 |

### 2.6 内存管理参数

```yaml
# 内存管理 (适用于Jetson Orin等嵌入式平台)
max_keyframes_with_cloud: 500     # 保留点云的最大关键帧数
enable_cloud_cleanup: true        # 启用自动清理
keyframe_cloud_res: 0.15          # 关键帧点云降采样分辨率

# 增量保存
enable_incremental_save: false    # 启用增量保存
incremental_save_interval: 500    # 每N个关键帧保存一次
incremental_save_path: "~/maps/incremental"   # 需修改为实际路径
```

**内存估算公式**：
```
估计内存(MB) ≈ (关键帧数 × 3MB) + (ikd-Tree点数 × 32B / 1MB)
```

对于8GB内存设备，建议 `max_keyframes_with_cloud` 设置为 500-1000。

### 2.7 重定位参数

```yaml
relocalization:
  enable_on_startup: false        # 启动时自动重定位
  prior_map_path: ""              # 先验地图路径
  sc_match_threshold: 0.25        # SC匹配阈值
  icp_refine_threshold: 0.5       # ICP精化阈值
  max_attempts: 10                # 最大尝试次数
  timeout_sec: 30.0               # 超时时间
  use_global_map_icp: true        # 使用全局地图进行ICP
```

### 2.8 场景配置建议

#### 室内小场景 (办公室、家庭)

```yaml
lidar_max_range: 15.0
cube_len: 100
det_range: 30
keyframe_dist_threshold: 0.5
loop_search_radius: 8.0
sc_dist_threshold: 0.15
```

#### 室内大场景 (商场、仓库)

```yaml
lidar_max_range: 30.0
cube_len: 200
det_range: 50
keyframe_dist_threshold: 1.0
loop_search_radius: 15.0
sc_dist_threshold: 0.20
```

#### 室外开阔场景

```yaml
lidar_max_range: 80.0
cube_len: 500
det_range: 100
keyframe_dist_threshold: 2.0
loop_search_radius: 25.0
sc_dist_threshold: 0.30
```

#### 动态环境 (行人较多)

```yaml
lidar_filter_num: 4
scan_resolution: 0.1
loop_time_diff_threshold: 60.0
icp_fitness_threshold: 0.4
```

---

## 3. 安装与部署

### 3.1 系统要求

| 组件 | 要求 |
|------|------|
| **操作系统** | Ubuntu 22.04 LTS |
| **ROS版本** | ROS 2 Humble |
| **CPU** | x86_64 或 ARM64 (Jetson Orin) |
| **内存** | >= 8GB (推荐16GB) |
| **存储** | >= 50GB SSD |

### 3.2 依赖安装


## 实例数据集
```text
链接: https://pan.baidu.com/s/1rTTUlVwxi1ZNo7ZmcpEZ7A?pwd=t6yb 提取码: t6yb 
--来自百度网盘超级会员v7的分享
```

#### 3.2.1 扩展swap，根据情况扩展，推荐8G及以上
```bash
sudo swapoff /swapfile 2>/dev/null
sudo dd if=/dev/zero of=/swapfile bs=1M count=8192 status=progress && \
sudo chmod 600 /swapfile && \
sudo mkswap /swapfile && \
sudo swapon /swapfile && \
free -h
```
#### 3.2.2 安装系统依赖

```bash
# 基础依赖
sudo apt install -y \
    cmake build-essential git \
    libeigen3-dev libpcl-dev \
    libboost-all-dev \
    libgoogle-glog-dev \
    python3-pip python3-tk

# ROS 2相关包
sudo apt install -y \
    ros-humble-pcl-conversions \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs
```

#### 3.2.3 安装GTSAM 4.2.0

> **重要**: Ubuntu 22.04 系统自带 Eigen 3.4.0，必须添加 `-DGTSAM_USE_SYSTEM_EIGEN=ON` 参数，否则会导致编译 fastlio2 时出现 Eigen 版本不匹配错误。

```bash
# 克隆GTSAM
cd ~
git clone --branch 4.2.0 https://github.com/borglab/gtsam.git
cd gtsam

# 编译安装 (注意: 必须使用 GTSAM_USE_SYSTEM_EIGEN=ON)
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j$(nproc)
sudo make install

# 配置动态链接库路径 (重要! 否则运行时会报 libgtsam.so.4 找不到)
echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/gtsam.conf
sudo ldconfig

# 验证安装
ldconfig -p | grep gtsam
```

#### 3.2.4 安装Sophus

```bash
cd ~
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
#### 3.2.5 安装livox sdk2
```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```
#### 3.2.6 安装Livox ROS2驱动
```bash
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/livox_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```


### 3.3 编译安装

```bash
# 克隆项目 (如果还没有)
cd ~
git clone https://github.com/your-username/fastlio2_ros2.git

# 编译
cd ~/fastlio2_ros2
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
colcon build --packages-select interface
source install/setup.bash
colcon build 

# 添加到环境
echo "source ~/fastlio2_ros2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.4 新用户配置 (重要!)

> **首次部署必读**: 以下文件包含硬编码路径，新用户必须根据自己的系统环境进行修改。

#### 3.4.1 配置文件路径修改

编辑 `fastlio2/config/lio.yaml`，修改以下路径参数：

```yaml
# 将 /home/li 替换为你的用户主目录，例如 /home/your_username

# 地图保存路径 (第275行附近)
map_save_path: "/home/li/maps"              # 修改为: "/home/your_username/maps"

# 增量保存路径 (第359行附近)
incremental_save_path: "/home/li/maps/incremental"  # 修改为: "/home/your_username/maps/incremental"

# 重定位先验地图路径 (第394行附近)
prior_map_path: "/home/li/maps"             # 修改为: "/home/your_username/maps"
```

**快速替换命令**:
```bash
# 将配置文件中的 /home/li 替换为当前用户目录
sed -i "s|/home/li|$HOME|g" ~/fastlio2_ros2/fastlio2/config/lio.yaml
```

#### 3.4.2 GUI脚本路径 (已自动检测)

GUI 脚本 (`scripts/gui_launcher.py`) 现在会**自动检测**工作空间路径，无需手动修改。

- `ws_path`: 根据脚本位置自动推断
- `livox_ws_path`: 默认使用 `~/livox_ws`

> **注意**: 如果你的 livox_ws 不在默认位置 (`~/livox_ws`)，需要手动修改 `scripts/gui_launcher.py` 第44行。

#### 3.4.3 创建地图保存目录

```bash
mkdir -p ~/maps/incremental
```

### 3.5 嵌入式平台配置

#### Jetson Orin优化配置

```bash
# 设置最大性能模式
sudo nvpmodel -m 0
sudo jetson_clocks

# 增加swap空间 (如果内存不足)
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

**推荐配置修改** (lio.yaml):

```yaml
# Jetson Orin 8GB配置
max_keyframes_with_cloud: 300
enable_cloud_cleanup: true
lidar_filter_num: 8
scan_resolution: 0.2
map_resolution: 0.4
enable_incremental_save: true
incremental_save_interval: 300
```

---

## 4. 运行指南

### 4.1 启动命令

#### 方式一：使用GUI控制面板 (推荐)

```bash
cd ~/fastlio2_ros2/scripts
python3 gui_launcher.py
```

#### 方式二：命令行启动

```bash
# 终端1: 启动Livox驱动
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端2: 启动FASTLIO2
source /opt/ros/humble/setup.bash
source ~/fastlio2_ros2/install/setup.bash
ros2 launch fastlio2 lio_launch.py
```

#### 播放ROS2数据包

```bash
ros2 bag play /path/to/your/rosbag --rate 1.0
```

### 4.2 数据流处理

```
┌─────────────────────────────────────────────────────────────────┐
│                      ROS2 话题数据流                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  输入话题:                                                        │
│  ┌──────────────────┐  ┌──────────────────┐                    │
│  │ /livox/imu       │  │ /livox/lidar     │                    │
│  │ sensor_msgs/Imu  │  │ CustomMsg        │                    │
│  └────────┬─────────┘  └────────┬─────────┘                    │
│           │                      │                              │
│           └──────────┬───────────┘                              │
│                      ▼                                          │
│            ┌─────────────────┐                                  │
│            │  /fastlio2/lio  │                                  │
│            │    lio_node     │                                  │
│            └────────┬────────┘                                  │
│                     │                                           │
│           ┌─────────┴─────────────────┬───────────────┐        │
│           ▼                           ▼               ▼        │
│  ┌─────────────────┐  ┌─────────────────┐  ┌────────────────┐  │
│  │ /fastlio2/      │  │ /fastlio2/      │  │ /fastlio2/     │  │
│  │ lio_path        │  │ world_cloud     │  │ lio_odom       │  │
│  │ nav_msgs/Path   │  │ PointCloud2     │  │ nav_msgs/Odom  │  │
│  └─────────────────┘  └─────────────────┘  └────────────────┘  │
│                                                                  │
│  服务:                                                           │
│  ┌─────────────────────────────────────┐                       │
│  │ /fastlio2/lio/save_map              │                       │
│  │ interface/srv/SaveMaps              │                       │
│  └─────────────────────────────────────┘                       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 4.3 关键帧管理机制

关键帧选择遵循两个条件 (满足任一即插入):

1. **距离条件**: 相对上一关键帧位移 >= `keyframe_dist_threshold` (默认1.0m)
2. **角度条件**: 相对上一关键帧旋转 >= `keyframe_angle_threshold` (默认0.2rad)

关键帧包含以下数据:
- 关键帧ID和时间戳
- 6DoF位姿 (旋转矩阵 + 位置向量)
- 降采样后的点云 (可选，受内存管理控制)
- ScanContext描述子

### 4.4 性能优化策略

#### 实时性优化

```yaml
# 减少计算量
lidar_filter_num: 8           # 增大抽样率
scan_resolution: 0.2          # 增大降采样粒度
ieskf_max_iter: 3             # 减少迭代次数

# 降低回环检测频率
loop_closure_frequency: 0.5   # 每2秒检测一次
```

#### 精度优化

```yaml
# 保留更多点云细节
lidar_filter_num: 3
scan_resolution: 0.1
near_search_num: 8

# 提高回环检测精度
sc_dist_threshold: 0.15
icp_fitness_threshold: 0.3
```

---

## 5. GUI界面功能

GUI控制面板采用标签页布局，提供直观的系统控制和监控功能。

### 5.1 纯建图标签页

**主界面布局**：左侧操作控制，右侧实时监控

#### 左侧控制区

| 控件 | 功能 |
|------|------|
| **一键启动全部** | 依次启动LIVOX驱动和FASTLIO2节点 |
| **停止全部** | 安全停止所有运行中的节点 |
| **启动LIVOX** | 单独启动Livox驱动 |
| **启动FASTLIO2** | 单独启动SLAM节点 |
| **数据包播放** | 选择rosbag路径，设置播放速率 |
| **数据录制** | 录制LiDAR和IMU数据到rosbag |
| **地图保存** | 保存全局地图和关键帧数据 |

#### 右侧监控区

| 监控项 | 说明 |
|--------|------|
| **关键帧数** | 当前已创建的关键帧总数 |
| **点云缓存** | 保留点云的关键帧数/最大限制 |
| **估计内存** | 系统估计的内存占用 |
| **前端耗时** | 单次IESKF更新耗时 |
| **回环次数** | 检测到的有效回环数 |
| **回环状态** | 回环检测功能启用状态 |
| **探测范围** | 当前配置的有效探测距离 |
| **地图分辨率** | 保存地图的体素分辨率 |

### 5.2 回环检测标签页

提供回环检测参数的可视化配置:

- **检测开关**: 启用/禁用回环检测
- **检测频率**: 回环检测线程执行频率
- **搜索半径**: 候选帧空间搜索范围
- **时间差阈值**: 避免临近帧误检
- **SC相似度阈值**: ScanContext匹配严格度
- **ICP适配度阈值**: 几何验证严格度
- **子图大小**: ICP匹配使用的关键帧数

### 5.3 重定位标签页

配置在已建地图中的重定位功能:

- **先验地图路径**: 选择之前保存的地图目录
- **地图完整性检查**: 验证SCD文件、位姿文件、点云地图
- **匹配参数**: SC匹配阈值、ICP精化阈值
- **运行参数**: 最大尝试次数、超时时间

### 5.4 高级工具标签页

包含三个子功能页:

#### 离线优化
- 加载已保存的pose_graph.g2o文件
- 执行批量位姿图优化
- 输出优化后的位姿和地图

#### Session合并
- 添加多个建图Session目录
- 设置合并输出路径
- 可选: 合并点云地图并降采样

#### 轨迹对比
- 加载优化前/后轨迹文件
- 生成对比分析报告
- 可视化轨迹差异

---

## 6. 高级功能特性

### 6.1 回环检测机制

FASTLIO2采用三级验证的回环检测策略:

```
Stage 1: 距离搜索 (KD-Tree)
├── 搜索范围: loop_search_radius (15m)
├── 排除最近N帧: 避免临近帧误检
└── 时间差过滤: loop_time_diff_threshold (30s)

Stage 2: ScanContext验证
├── 计算当前帧SC描述子
├── 与候选帧SC进行匹配
├── 阈值: sc_dist_threshold (0.20)
└── 输出: 粗略旋转估计 (yaw_offset)

Stage 3: ICP几何验证
├── 构建source/target子图 (submap_size帧)
├── 使用yaw_offset作为初始估计
├── 执行ICP配准
├── 阈值: icp_fitness_threshold (0.5)
└── 输出: 精确相对位姿变换
```

**ScanContext描述子**:
- 将点云投影到极坐标系
- 创建20×60的ring-sector矩阵
- 旋转不变性: 列位移匹配
- 计算距离: 余弦距离

### 6.2 重定位功能

重定位允许在已建地图中确定初始位姿:

**使用流程**:

1. **准备先验地图**: 确保包含以下文件
   ```
   prior_map/
   ├── scd/                    # ScanContext描述子
   │   ├── 0000.scd
   │   ├── 0001.scd
   │   └── ...
   ├── optimized_pose.txt      # KITTI格式位姿
   └── GlobalMap.pcd           # 全局点云地图 (可选)
   ```

2. **配置参数**: 修改lio.yaml
   ```yaml
   relocalization:
     enable_on_startup: true
     prior_map_path: "~/maps/session1"   # 修改为你的地图路径
   ```

3. **启动系统**: 系统将自动执行重定位
   - 使用ScanContext查找最相似的先验关键帧
   - 使用ICP精化位姿估计
   - 成功后以先验地图坐标系为参考继续建图

### 6.3 增量地图保存

对于长时间运行任务，启用增量保存避免数据丢失:

```yaml
enable_incremental_save: true
incremental_save_interval: 500    # 每500个关键帧
incremental_save_path: "~/maps/incremental"   # 修改为你的路径
```

保存内容:
- `patches/`: 关键帧点云 (*.pcd)
- `scd/`: ScanContext描述子 (*.scd)
- `optimized_pose.txt`: 当前优化位姿
- `pose_graph.g2o`: 位姿图文件

### 6.4 Session合并

使用 `merge_sessions.py` 脚本合并多次建图结果:

```bash
python3 scripts/merge_sessions.py \
    --sessions "/path/to/session1" "/path/to/session2" \
    --output "/path/to/merged" \
    --merge-pcd \
    --voxel-size 0.1
```

合并策略:
1. 加载所有Session的位姿和点云
2. 对齐坐标系 (使用最后一帧与下一Session首帧的重叠)
3. 合并位姿图
4. 可选: 合并点云地图并降采样

---

## 7. 故障排除

### 问题1: 编译错误 - 找不到GTSAM

**错误信息**:
```
Could not find a package configuration file provided by "GTSAM"
```

**解决方案**:
```bash
# 确认GTSAM安装位置
ls /usr/local/lib/cmake/GTSAM/

# 如果不存在，重新安装GTSAM
cd ~/gtsam/build
sudo make install

# 或手动指定路径
export GTSAM_DIR=/usr/local/lib/cmake/GTSAM
```

### 问题2: 编译错误 - GTSAM与Eigen版本不匹配

**错误信息**:
```
error: static assertion failed: Error: GTSAM was built against a different version of Eigen
```

**原因**: GTSAM 默认使用自带的 Eigen 3.3.7，但系统安装的是 Eigen 3.4.0（Ubuntu 22.04 默认版本）。

**解决方案**:
```bash
# 1. 删除旧的 GTSAM 安装
sudo rm -rf /usr/local/include/gtsam
sudo rm -rf /usr/local/lib/libgtsam*
sudo rm -rf /usr/local/lib/cmake/GTSAM

# 2. 重新编译 GTSAM (必须使用系统 Eigen)
cd ~/gtsam/build
rm -rf *
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON
make -j$(nproc)
sudo make install

# 3. 验证安装 (应显示 GTSAM_EIGEN_VERSION_MAJOR 为 4)
grep "GTSAM_EIGEN_VERSION" /usr/local/include/gtsam/config.h

# 4. 重新编译 fastlio2_ros2
cd ~/fastlio2_ros2
rm -rf build install log
colcon build --packages-select interface
colcon build
```

### 问题3: 运行时找不到GTSAM动态库

**错误信息**:
```
error while loading shared libraries: libgtsam.so.4: cannot open shared object file: No such file or directory
```

**原因**: GTSAM 安装到 `/usr/local/lib`，但系统动态链接器未包含该路径。

**解决方案**:
```bash
# 添加 /usr/local/lib 到动态链接器搜索路径
echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/gtsam.conf
sudo ldconfig

# 验证
ldconfig -p | grep gtsam
```

### 问题4: 运行时找不到livox_ros_driver2库

**错误信息**:
```
error while loading shared libraries: liblivox_ros_driver2...
```

**解决方案**:
```bash
# 确保source了livox工作空间
source ~/livox_ws/install/setup.bash

# 添加到ldconfig
echo "/home/$USER/livox_ws/install/livox_ros_driver2/lib" | sudo tee /etc/ld.so.conf.d/livox.conf
sudo ldconfig
```

### 问题5: IMU数据时间戳乱序

**错误信息**:
```
[WARN] IMU Message is out of order
```

**解决方案**:
- 检查传感器时钟同步
- 确保数据包录制时ROS2 QoS配置正确
- 尝试降低数据包播放速率: `ros2 bag play xxx --rate 0.5`

### 问题6: 回环检测误匹配导致地图错误

**现象**: 地图出现明显扭曲或错位

**解决方案**:
```yaml
# 提高回环检测严格度
sc_dist_threshold: 0.15        # 减小 (更严格)
icp_fitness_threshold: 0.3     # 减小 (更严格)
loop_time_diff_threshold: 60.0 # 增大 (更大时间间隔)
```

### 问题7: 内存不足导致系统崩溃

**解决方案**:
```yaml
# 减少内存占用
max_keyframes_with_cloud: 200
enable_cloud_cleanup: true
lidar_filter_num: 8
scan_resolution: 0.25
enable_incremental_save: true
```

### 问题8: GUI启动失败

**错误信息**:
```
ModuleNotFoundError: No module named 'tkinter'
```

**解决方案**:
```bash
sudo apt install python3-tk
```

---

## 8. 性能评估

### 8.1 时间复杂度分析

| 模块 | 时间复杂度 | 典型耗时 |
|------|-----------|---------|
| 点云预处理 | O(n) | 2-5 ms |
| ikd-Tree近邻搜索 | O(log n) | 5-10 ms |
| IESKF单次迭代 | O(n) | 3-8 ms |
| IESKF总更新 (5次) | O(5n) | 15-40 ms |
| ScanContext生成 | O(n) | 1-2 ms |
| ScanContext匹配 | O(k) | 0.1-0.5 ms |
| ICP配准 | O(mn) | 50-200 ms |
| GTSAM优化 | O(k²) | 10-50 ms |

其中: n=点云数量, k=关键帧数量, m=ICP迭代次数

### 8.2 硬件资源消耗

#### x86_64 平台 (Intel i7-10700)

| 指标 | 数值 |
|------|------|
| CPU使用率 | 30-50% (单核) |
| 内存占用 | 2-4 GB (500关键帧) |
| 前端频率 | 20-30 Hz |

#### ARM64 平台 (Jetson Orin 8GB)

| 指标 | 数值 |
|------|------|
| CPU使用率 | 50-70% |
| 内存占用 | 2-3 GB (优化配置) |
| 前端频率 | 15-20 Hz |
| GPU使用率 | 未使用 |

### 8.3 精度评估

在典型室内环境 (500m²) 测试结果:

| 指标 | 数值 |
|------|------|
| 位置漂移 (无回环) | 0.5-1.0% |
| 位置漂移 (有回环) | 0.1-0.3% |
| 回环检测召回率 | 85-95% |
| 回环检测精度 | 95-99% |

---

## 附录

### A. 输出文件说明

| 文件 | 说明 |
|------|------|
| `GlobalMap.pcd` | 全局点云地图 |
| `patches/*.pcd` | 各关键帧点云 |
| `scd/*.scd` | ScanContext描述子 |
| `optimized_pose.txt` | 优化后位姿 (KITTI格式) |
| `without_optimized_pose.txt` | 优化前位姿 |
| `pose_graph.g2o` | G2O格式位姿图 |
| `fast_lio_time_log.csv` | 性能日志 |

### B. KITTI位姿格式

每行12个数字，表示3×4的变换矩阵 [R|t]:
```
r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
```

### C. 常用ROS2命令

```bash
# 查看话题
ros2 topic list
ros2 topic echo /fastlio2/lio_odom

# 调用保存服务
ros2 service call /fastlio2/lio/save_map interface/srv/SaveMaps \
  "{file_path: '~/maps/test', save_patches: true}"

# 记录数据包
ros2 bag record -o my_bag /livox/lidar /livox/imu
```

---

**项目维护**: FASTLIO2 ROS2 Team  
**许可证**: MIT License  
**版本**: 3.0
