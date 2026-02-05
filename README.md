# FASTLIO2 ROS2
## 主要工作
1. 重构[FASTLIO2](https://github.com/hku-mars/FAST_LIO) 适配ROS2
2. 添加回环节点，基于位置先验+ICP进行回环检测，基于GTSAM进行位姿图优化
3. 添加重定位节点，基于由粗到细两阶段ICP进行重定位
4. 增加一致性地图优化，基于[BLAM](https://github.com/hku-mars/BALM) (小场景地图) 和[HBA](https://github.com/hku-mars/HBA) (大场景地图)

## 环境依赖
1. Ubuntu 22.04
2. ROS2 Humble

## 编译依赖
```text
pcl
Eigen
sophus
gtsam
livox_ros_driver2
```

## 详细说明
### 1.编译 LIVOX-SDK2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

### 2.编译 livox_ros_driver2
```shell
mkdir -r ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```

### 3.编译 Sophus
```shell
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```

**新的Sophus依赖fmt，可以在CMakeLists.txt中添加add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)去除，否则会报错**


## 实例数据集
```text
链接: https://pan.baidu.com/s/1rTTUlVwxi1ZNo7ZmcpEZ7A?pwd=t6yb 提取码: t6yb 
--来自百度网盘超级会员v7的分享
```

## 部分脚本

### 1.激光惯性里程计 
```shell
ros2 launch fastlio2 lio_launch.py
ros2 bag play your_bag_file
```

### 2.里程计加回环
#### 启动回环节点
```shell
ros2 launch pgo pgo_launch.py
ros2 bag play your_bag_file
```
#### 保存地图
```shell
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: 'your_save_dir', save_patches: true}"
```

### 3.里程计加重定位
#### 启动重定位节点
```shell
ros2 launch localizer localizer_launch.py
ros2 bag play your_bag_file // 可选
```
#### 设置重定位初始值
```shell
ros2 service call /localizer/relocalize interface/srv/Relocalize "{"pcd_path": "your_map.pcd", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
```
#### 检查重定位结果
```shell
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
```

### 4.一致性地图优化
#### 启动一致性地图优化节点
```shell
ros2 launch hba hba_launch.py
```
#### 调用优化服务
```shell
ros2 service call /hba/refine_map interface/srv/RefineMap "{"maps_path": "your maps directory"}"
```
**如果需要调用优化服务，保存地图时需要设置save_patches为true**

## 特别感谢
1. [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
2. [BLAM](https://github.com/hku-mars/BALM)
3. [HBA](https://github.com/hku-mars/HBA)
## 性能相关的问题
该代码主要使用timerCB作为频率触发主函数，由于ROS2中的timer、subscriber以及service的回调实际上运行在同一个线程上，在电脑性能不是好的时候，会出现调用阻塞的情况，建议使用线程并发的方式将耗时的回调独立出来(如timerCB)来提升性能

---

# 防漂移功能模块（实时回环闭合）

## 功能概述

本模块在FAST-LIO2的ESKF框架基础上，集成了实时回环检测与因子图优化能力，用于消除长时间运行中的累积漂移。整体架构采用**分层设计**：

```
┌─────────────────────────────────────────────────────────────────────┐
│                        FAST-LIO2 主节点 (lio_node)                   │
├─────────────────────────────────────────────────────────────────────┤
│  ESKF状态估计 (50Hz)  ←────────── 状态修正 ←─────────┐              │
│         ↓                                            │              │
│  关键帧判断 (距离+角度)                              │              │
│         ↓                                            │              │
│  ┌──────────────────────────────────────────────────┴─────────┐    │
│  │              回环检测线程 (1Hz, 独立线程)                    │    │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────────┐ │    │
│  │  │ 关键帧管理器 │ → │ 回环检测器  │ → │ 因子图优化器    │ │    │
│  │  │ KeyFrameMgr │    │ LoopDetector│    │ PoseGraphOptim │ │    │
│  │  └─────────────┘    └─────────────┘    └─────────────────┘ │    │
│  └────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────┘
```

### 核心模块说明

| 模块 | 文件 | 功能 |
|------|------|------|
| **关键帧管理器** | `src/loop_closure/keyframe_manager.h/cpp` | 基于位姿增量判断关键帧，管理关键帧点云和位姿 |
| **回环检测器** | `src/loop_closure/loop_detector.h/cpp` | 三级验证：KD-Tree候选 → ScanContext相似度 → ICP精配准 |
| **因子图优化器** | `src/loop_closure/pose_graph_optimizer.h/cpp` | GTSAM ISAM2增量优化，管理先验/里程计/回环因子 |
| **状态反馈** | `src/map_builder/ieskf.h/cpp` | 新增`change_x()`接口，将优化结果反馈到ESKF状态 |

### 回环检测流程

```
1. KD-Tree距离候选
   └── 在历史关键帧位置中搜索半径内的候选（排除时间过近的帧）
   
2. ScanContext相似度验证
   └── 计算极坐标BEV描述子相似度，估计初始yaw偏移
   
3. ICP精配准验证
   └── 使用SC估计的yaw预对齐，执行ICP配准
   └── 根据fitness score判断回环有效性
   
4. 因子图优化
   └── 添加回环约束因子，ISAM2增量优化
   └── 更新所有关键帧位姿
   
5. 状态反馈
   └── 计算当前帧的位姿修正量
   └── 通过change_x()接口修正ESKF状态
```

## 新增依赖

除原有依赖外，防漂移功能需要：

```text
GTSAM 4.2.0 (本地安装，非ROS2内置版本)
nanoflann (已包含，无需额外安装)
ScanContext (已包含，无需额外安装)
```

### GTSAM 4.2.0 安装

**重要**: 必须使用本地安装的GTSAM 4.2.0，不能使用ROS2 Humble内置版本（存在兼容性问题）

```shell
# 下载GTSAM 4.2.0
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.2.0

# 编译安装
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF
make -j$(nproc)
sudo make install

# 验证安装
ls /usr/local/lib/cmake/GTSAM/
```

## 配置参数说明

在 `config/lio.yaml` 中新增以下参数：

### 关键帧参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `keyframe_dist_threshold` | 1.0 | 关键帧距离阈值(米)，相邻关键帧间移动距离超过此值时添加新关键帧 |
| `keyframe_angle_threshold` | 0.2 | 关键帧角度阈值(弧度)，相邻关键帧间旋转角度超过此值时添加新关键帧 |

### 回环检测参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `loop_closure_enable` | false | 是否启用回环检测功能 |
| `loop_closure_frequency` | 1.0 | 回环检测线程频率(Hz) |
| `loop_search_radius` | 10.0 | 回环候选搜索半径(米) |
| `loop_time_diff_threshold` | 30.0 | 回环候选最小时间差(秒)，防止误检测相邻帧 |
| `sc_dist_threshold` | 0.3 | ScanContext相似度阈值，越小要求越相似 |
| `icp_fitness_threshold` | 0.5 | ICP配准fitness score阈值，越小要求配准越好 |
| `submap_size` | 25 | 构建子图时使用的关键帧数量 |

### 配置示例

```yaml
# =========== 防漂移功能配置 ===========
# 关键帧选取
keyframe_dist_threshold: 1.0        # 米，建议室内0.5-1.0，室外1.0-2.0
keyframe_angle_threshold: 0.2       # 弧度(约11.5度)

# 回环检测
loop_closure_enable: true           # 设为true启用回环检测
loop_closure_frequency: 1.0         # Hz，回环检测频率
loop_search_radius: 15.0            # 米，根据场景大小调整
loop_time_diff_threshold: 30.0      # 秒，避免误检测
sc_dist_threshold: 0.25             # ScanContext阈值，室内可适当放宽到0.3-0.4
icp_fitness_threshold: 0.3          # ICP阈值，室内可适当放宽到0.5

# 子图构建
submap_size: 25                     # 关键帧数量，影响回环验证精度
```

## 使用方法

### 1. 编译

```shell
cd /home/li/FASTLIO2_ROS2
source /opt/ros/humble/setup.bash
colcon build --packages-select fastlio2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**编译注意事项**：
- CMakeLists.txt中已硬指定GTSAM路径：`set(GTSAM_DIR "/usr/local/lib/cmake/GTSAM")`
- 如果GTSAM安装路径不同，请修改此配置

### 2. 启用回环检测

编辑 `config/lio.yaml`：

```yaml
loop_closure_enable: true
```

### 3. 运行

```shell
# 终端1: 启动节点
cd /home/li/FASTLIO2_ROS2
source /opt/ros/humble/setup.bash
source /home/li/ws_livox/install/setup.bash
source install/setup.bash
ros2 launch fastlio2 lio_launch.py
# 终端2: 播放数据或实时运行
source /home/li/ws_livox/install/setup.bash
ros2 bag play your_bag_with_loop.db3
```

### 4. 验证回环检测

观察终端输出，当检测到回环时会打印：

```
[INFO] [lio_node]: === Loop Closure Detected ===
[INFO] [lio_node]: Current frame: 150, Loop frame: 23
[INFO] [lio_node]: ICP fitness score: 0.125
[INFO] [lio_node]: Applying loop correction...
[INFO] [lio_node]: Pose correction applied successfully
```

### 5. 保存地图

运行结束后，调用服务保存地图：

```shell
# 保存全局地图（合并所有关键帧点云，降采样后保存）
ros2 service call /lio/save_map interface/srv/SaveMaps "{file_path: '/home/li/maps/my_map', save_patches: false}"

# 保存全局地图 + 各关键帧独立点云（用于后续优化）
ros2 service call /lio/save_map interface/srv/SaveMaps "{file_path: '/home/li/maps/my_map', save_patches: true}"
```

**输出文件说明**：
- `global_map.pcd`：合并并降采样后的全局地图（0.1m分辨率）
- `patch_0.pcd`, `patch_1.pcd`, ...：各关键帧的独立点云（仅当`save_patches: true`时生成）

## 关键代码修改说明

### 1. ESKF状态修正接口 (`src/map_builder/ieskf.h/cpp`)

新增两个公有方法：

```cpp
// 直接修改位姿状态（保留速度和IMU偏置）
void change_x(const M3D& rotation, const V3D& position);

// 重置位姿协方差（回环修正后稳定滤波器）
void resetPoseCovariance(double variance = 1e-4);
```

**设计原理**：
- `change_x()`仅修改旋转和平移，保留速度和IMU偏置，避免滤波器发散
- `resetPoseCovariance()`在回环修正后将位姿协方差重置为较小值，使滤波器信任优化结果

### 2. 回环检测线程 (`src/lio_node.cpp`)

在LIONode类中新增独立线程处理回环检测：

```cpp
// 线程函数
void loopClosureThread() {
    while (m_loop_thread_running) {
        // 1. 检测回环
        auto result = m_loop_detector->detectLoopClosure(current_kf, keyframes);
        
        // 2. 如果检测到有效回环
        if (result.valid) {
            // 添加回环因子并优化
            m_pose_graph->addLoopFactor(...);
            m_pose_graph->optimize();
            
            // 更新关键帧位姿
            updateKeyFramePoses();
            
            // 修正ESKF状态
            applyLoopCorrection();
        }
        
        // 3. 按配置频率休眠
        std::this_thread::sleep_for(period);
    }
}
```

### 3. 因子图结构 (`src/loop_closure/pose_graph_optimizer.cpp`)

```
因子图结构:
├── X0: PriorFactor (第一帧固定，噪声极小 1e-12)
├── X0-X1: BetweenFactor (里程计约束，噪声 rot:1e-6, trans:1e-4)
├── X1-X2: BetweenFactor
├── ...
├── Xi-Xj: BetweenFactor (回环约束，噪声基于ICP fitness score)
└── ...
```

**噪声模型设计**：
- 先验因子：1e-12，几乎固定第一帧
- 里程计因子：旋转1e-6，平移1e-4，信任ESKF估计
- 回环因子：直接使用ICP fitness score作为噪声，自适应约束强度

## 调试与故障排查

### 问题1: 编译报错 `metis-gtsam` 库不存在

**错误信息**：
```
The imported target "metis-gtsam" references the file 
"/opt/ros/humble/lib/x86_64-linux-gnu/libmetis-gtsam.so" but this file does not exist.
```

**原因**：CMake找到了ROS2内置的GTSAM配置，而非本地安装的4.2.0版本

**解决方案**：确保CMakeLists.txt中有以下配置：
```cmake
set(GTSAM_DIR "/usr/local/lib/cmake/GTSAM")
find_package(GTSAM REQUIRED)
```

### 问题2: 回环检测无输出

**检查步骤**：
1. 确认 `loop_closure_enable: true`
2. 检查关键帧数量是否足够：至少需要积累一定数量的关键帧才能检测回环
3. 检查 `loop_time_diff_threshold`：如果设置过大，可能导致候选帧被过滤
4. 检查 `loop_search_radius`：如果设置过小，可能无法找到候选帧

### 问题3: 回环检测频繁误检

**调整参数**：
```yaml
sc_dist_threshold: 0.2          # 降低阈值，要求更高相似度
icp_fitness_threshold: 0.3      # 降低阈值，要求更好配准
loop_time_diff_threshold: 60.0  # 增大时间差，避免相近帧误检
```

### 问题4: 回环修正后轨迹跳变

**可能原因**：回环约束与里程计约束冲突过大

**调整方案**：
1. 检查回环检测是否为误检测
2. 适当放宽里程计因子噪声（在pose_graph_optimizer.cpp中调整）
3. 增大回环因子噪声（提高icp_fitness_threshold）

## 性能说明

| 指标 | 典型值 | 说明 |
|------|--------|------|
| 关键帧频率 | 1-5 Hz | 取决于运动速度和阈值设置 |
| 回环检测耗时 | 50-200 ms | 主要耗时在ICP配准 |
| 内存增长 | ~5 MB/分钟 | 关键帧点云存储 |
| ISAM2优化耗时 | 1-10 ms | 增量优化，随图规模增长 |

**线程模型**：
- 主线程：ESKF状态估计 @ 50Hz
- 回环线程：回环检测与优化 @ 1Hz（可配置）
- 两线程通过互斥锁保护共享数据（关键帧列表、优化结果）

## 文件结构

```
fastlio2/
├── src/
│   ├── loop_closure/                    # 新增：防漂移模块
│   │   ├── keyframe_manager.h/cpp       # 关键帧管理
│   │   ├── loop_detector.h/cpp          # 回环检测
│   │   └── pose_graph_optimizer.h/cpp   # 因子图优化
│   ├── map_builder/
│   │   └── ieskf.h/cpp                  # 修改：新增change_x()接口
│   └── lio_node.cpp                     # 修改：集成回环检测线程
├── include/
│   ├── sc-relo/                         # 新增：ScanContext库
│   │   ├── Scancontext.h/cpp
│   │   └── KDTreeVectorOfVectorsAdaptor.h
│   ├── nanoflann.hpp                    # 新增：KD-Tree库
│   └── tictoc.hpp                       # 新增：计时工具
├── config/
│   └── lio.yaml                         # 修改：新增回环检测配置
└── CMakeLists.txt                       # 修改：GTSAM链接、源文件添加
```

## 参考资料

- [fast_lio_sam](https://github.com/engcang/FAST_LIO_SAM) - 本模块的ScanContext回环检测参考
- [GTSAM](https://github.com/borglab/gtsam) - 因子图优化库
- [ScanContext](https://github.com/gisbi-kim/scancontext) - 场景描述子回环检测

# fastlio2_ros2
