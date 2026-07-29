# 3d_nav: TARE / HPHS on Unitree A1

本工作区按照 `说明文档.txt` 的需求，把 TARE 和 HPHS 原本基于小车的 Gazebo 仿真平台，替换为 3d-navi 中的 Unitree A1，并在 `Building.world` 场景中跑通。

当前已完成并验证：

- Unitree A1 Gazebo 仿真启动。
- A1 机身挂载 VLP-16，发布 `/velodyne_points`。
- 新增 `a1_exploration_bridge` 桥接包，将 A1/Gazebo 数据适配到 CMU exploration / TARE / HPHS 所需话题。
- TARE + A1 + Building 一键启动。
- HPHS + A1 + Building 一键启动，包含 `octomap_server`、`move_base` 和 `hphs_explorer`。
- README 中的构建与运行命令已在 `ros-noetic` Docker 容器中实测。

## 目录结构

```text
third_party/tare_planner
  TARE 官方上层探索算法。

third_party/HPHS
  HPHS 官方代码。已做 Noetic/A1 集成兼容修改。

third_party/3d-navi
  3d-navi 参考文件来源目录。

third_party/unitree_ros-master
  Unitree Gazebo、A1 描述、控制器和 Building 场景。

third_party/unitree_guide_upstream
  Unitree guide 和 unitree_move_base。

third_party/unitree_ros_to_real_upstream
  unitree_legged_msgs。

src/a1_exploration_bridge
  本项目新增 ROS 包：桥接节点和一键启动 launch。

src/vendor
  catkin 工作区入口，主要是指向 third_party 源码的软链接。
```

Building 场景文件：

```text
third_party/unitree_ros-master/unitree_gazebo/worlds/Building.world
third_party/unitree_ros-master/unitree_gazebo/models/Building/model.sdf
```

注意：原始 `Building.dae` 从 Gitee 下载时返回 `403`，当前 `model.sdf` 使用简化室内结构兜底，保证仿真和探索链路完整跑通。拿到原始 mesh 后，可替换回真实 Building 模型。

## Docker 环境

使用已有 ROS1 Noetic Docker：

```bash
docker run -it --name ros-noetic --net=host \
  -v /home/wya/nav:/workspace \
  osrf/ros:noetic-desktop-full bash
```

容器已存在时：

```bash
docker start ros-noetic
docker exec -it ros-noetic bash
```

容器内需要的 ROS 依赖：

```bash
apt-get update
apt-get install -y \
  ros-noetic-tf2-sensor-msgs \
  ros-noetic-move-base-msgs \
  ros-noetic-move-base \
  ros-noetic-octomap-server
```

当前 `ros-noetic` 容器中这些依赖已经安装并验证。Ubuntu apt 源在当前网络环境下可能不可用，但 ROS snapshot 源可用。

## 编译

容器内执行：

```bash
cd /workspace/3d_nav
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

本次检查结果：

```text
catkin_make: 通过
catkin packages: 24 个
Python py_compile: 通过
rospack find: tare_planner / HPHS / unitree_gazebo / a1_exploration_bridge 均通过
roslaunch --nodes / --files: bridge、A1、TARE、HPHS launch 均可解析
```

编译时会看到上游包警告，当前不阻断运行：

- `HPHS` 包名大写，不符合 catkin 命名规范。
- VTK imported target 缺少部分可执行文件。
- TARE `or-tools/lib` 可能遮蔽系统 `libz.so` 的 linker search path 警告。
- Gazebo classic 相关包有 deprecation 提示。

## 运行 TARE

容器内执行：

```bash
cd /workspace/3d_nav
source devel/setup.bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false
```

打开 GUI / RViz 时可改为：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=true headless:=false paused:=false rviz:=true
```

本次 TARE 实测结果：

```text
节点在线：
  /gazebo
  /cmu_a1_bridge
  /localPlanner
  /pathFollower
  /terrainAnalysis
  /sensorScanGeneration
  /sensor_coverage_planner/tare_planner_node
  /a1_gazebo/controller_spawner
  /robot_state_publisher

关键话题：
  /registered_scan  约 10 Hz
  /scan             约 10 Hz
  /terrain_map      约 9-10 Hz
  /sensor_scan      约 10 Hz
  /way_point        约 1 Hz
  /cmd_vel          约 50 Hz

样例：
  /way_point frame_id = map
  /cmd_vel angular.z = 1.2
```

## 运行 HPHS

容器内执行：

```bash
cd /workspace/3d_nav
source devel/setup.bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false
```

打开 GUI / RViz 时可改为：

```bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=true headless:=false paused:=false rviz:=true
```

本次 HPHS 实测结果：

```text
节点在线：
  /gazebo
  /cmu_a1_bridge
  /hphs_explorer
  /localPlanner
  /move_base
  /octomap_server
  /pathFollower
  /terrainAnalysis
  /sensorScanGeneration
  /a1_gazebo/controller_spawner
  /robot_state_publisher

关键话题：
  /registered_scan            约 10 Hz
  /scan                       约 10 Hz，发布者 /cmu_a1_bridge，订阅者 /move_base
  /terrain_map                约 10 Hz
  /sensor_scan                约 10 Hz
  /map                        约 10 Hz
  /state_estimation           约 1000 Hz
  /way_point                  有持续输出
  /cmu_cmd_vel                约 50 Hz
  /cmd_vel                    约 50 Hz

样例：
  /way_point frame_id = map
  /cmu_cmd_vel frame_id = vehicle
  /cmd_vel angular.z = 1.2
```

`roswtf` 复测结果：

```text
Static checks: 无错误、无警告
Online checks: /scan 未连接问题已修复
剩余 warning: /joy、/speed、/stop 等人工控制/可选输入未连接；Gazebo 控制器调试输入未连接；urdf_spawner 启动后退出
剩余 error: move_base footprint 自连接检查提示未连接，但 rostopic info 显示 footprint 由 move_base 发布并被 move_base 订阅，当前不影响运行
```

## 桥接节点

桥接节点文件：

```text
src/a1_exploration_bridge/scripts/cmu_a1_bridge.py
```

默认话题转换：

| 输入 | 输出 | 类型 | 说明 |
| --- | --- | --- | --- |
| `/gazebo/model_states` | `/state_estimation` | `nav_msgs/Odometry` | 从 Gazebo A1 模型状态生成 CMU 栈需要的里程计 |
| `/velodyne_points` | `/registered_scan` | `sensor_msgs/PointCloud2` | 清洗 Velodyne 点云并转换到 `map` frame |
| `/velodyne_points` | `/scan` | `sensor_msgs/LaserScan` | 给 HPHS `move_base` costmap 使用的 2D scan |
| `/cmu_cmd_vel` | `/cmd_vel` | `geometry_msgs/Twist` | 将 CMU `TwistStamped` 限幅后转给 A1/Gazebo |

重要参数在：

```text
src/a1_exploration_bridge/launch/bridge.launch
```

常用参数：

```bash
roslaunch a1_exploration_bridge bridge.launch \
  state_source:=gazebo_model_states \
  model_name:=a1_gazebo \
  scan_in:=/velodyne_points \
  registered_scan_out:=/registered_scan \
  laserscan_out:=/scan \
  transform_scan_to_map:=true \
  max_linear:=0.8 \
  max_angular:=1.2
```

## 验收命令

构建检查：

```bash
cd /workspace/3d_nav
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -m py_compile \
  src/a1_exploration_bridge/scripts/cmu_a1_bridge.py \
  third_party/HPHS/scripts/explorer.py
```

包检查：

```bash
rospack find tare_planner
rospack find HPHS
rospack find unitree_gazebo
rospack find a1_exploration_bridge
```

launch 解析检查：

```bash
roslaunch a1_exploration_bridge bridge.launch --nodes
roslaunch a1_exploration_bridge a1_building_sim.launch --nodes
roslaunch a1_exploration_bridge tare_a1_building.launch --nodes
roslaunch a1_exploration_bridge hphs_a1_building.launch --nodes
```

运行时话题检查：

```bash
rostopic hz /registered_scan
rostopic hz /scan
rostopic hz /terrain_map
rostopic hz /sensor_scan
rostopic echo -n 1 /way_point
rostopic echo -n 1 /cmd_vel
```

HPHS 额外检查：

```bash
rostopic info /scan
rostopic hz /map
rostopic info /move_base/NavfnROS/plan
roswtf
```

## 常见问题

### 重复启动失败

如果上一次 launch 没有正常退出，可能出现 Gazebo entity 已存在、ROS 节点重名或端口状态残留。直接清理容器运行态：

```bash
docker restart ros-noetic
docker exec -it ros-noetic bash
cd /workspace/3d_nav
source devel/setup.bash
```

### HPHS 有 TF extrapolation 日志

HPHS 的 `move_base` 在仿真时间下偶尔会打印类似：

```text
Lookup would require extrapolation ... from frame [vehicle] to frame [map]
Could not transform the global plan to the frame of the controller
```

当前实测中 `/map`、`/way_point`、`/cmu_cmd_vel`、`/cmd_vel` 都持续发布，所以该日志不阻断整体运行。它来自 `map -> sensor -> vehicle` TF 链和 `move_base` 控制周期之间的毫秒级时间戳差异。

### `urdf_spawner` 显示 died

`urdf_spawner` 负责把 A1 URDF spawn 到 Gazebo。成功 spawn 后进程退出是正常现象。

### `ps3joy` 被跳过

`src/vendor/joystick_drivers/ps3joy` 因容器缺少 `libusb-dev` 被 `CATKIN_IGNORE` 跳过。当前自动探索仿真不依赖 PS3 手柄节点。

### HPHS 不再需要 `pyquaternion`

HPHS 原脚本依赖 `pyquaternion`，本工作区已把唯一用到的 Z 轴旋转替换为 `numpy.sin/cos`，避免在容器里额外安装 pip 包。

## 参考项目

- TARE: <https://github.com/caochao39/tare_planner>
- HPHS: <https://github.com/bit-lsj/HPHS>
- 3d-navi: <https://gitee.com/fdsf3e2342/3d-navi>
- Unitree ROS: <https://github.com/unitreerobotics/unitree_ros>
- Unitree Guide: <https://github.com/unitreerobotics/unitree_guide>
- CMU Exploration: <https://www.cmu-exploration.com/>
