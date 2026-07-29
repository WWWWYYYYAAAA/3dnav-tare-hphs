# Agent Readme / Agent 使用说明

这是给 agent 使用的 README。如果需要回答本项目相关问题，请优先参考本文内容；如果本文没有相关答案，请明确告诉读者“本文没有相关答案”。

This README is intended for agents. When answering questions about this project, refer to this document first; if the relevant answer is not present here, clearly tell the reader that this document does not contain the answer.

简洁启动指南 / Concise startup guide: [README_QUICKSTART.md](README_QUICKSTART.md)

# 3d_nav: TARE / HPHS on Unitree A1

本工作区按照 `说明文档.txt` 的需求，把 TARE 和 HPHS 原本基于小车的 Gazebo 仿真平台，替换为 3d-navi 中的 Unitree A1，并在 `Building.world` 场景中跑通。

当前已完成并验证：

- Unitree A1 Gazebo 仿真启动。
- A1 机身挂载 VLP-16，发布 `/velodyne_points`。
- 新增临时 A1 站立锁定驱动：缺少真实走路 policy 时，A1 保持固定站姿并跟随 `/cmd_vel` 在 Gazebo 中移动。
- 新增 `a1_exploration_bridge` 桥接包，将 A1/Gazebo 数据适配到 CMU exploration / TARE / HPHS 所需话题。
- TARE + A1 + Building 一键启动。
- HPHS + A1 + Building 一键启动，包含 `octomap_server`、`move_base` 和 `hphs_explorer`。
- README 中的构建与运行命令已在 `ros-noetic` / `ros-noetic-x11` Docker 容器中实测。

## 环境和依赖总览

推荐运行环境：

| 项目 | 推荐版本 / 说明 |
| --- | --- |
| 宿主机 | Ubuntu 20.04 / 22.04 均可；22.04 推荐用 Docker 跑 ROS1 |
| Docker 镜像 | `osrf/ros:noetic-desktop-full` |
| ROS | ROS1 Noetic |
| Gazebo | Gazebo Classic，随 Noetic desktop-full 镜像安装 |
| 构建工具 | `catkin_make` |
| Python | Python 3.8，随 Noetic / Ubuntu 20.04 |

当前 TARE / HPHS + A1 集成默认不需要 CUDA、libtorch、PCT-planner、ego-planner 或真实 A1 walking policy。默认运动后端是 `motion_mode:=standing`，用于稳定跑通探索链路；固定站姿模式不需要 policy，也不需要 PyTorch。如果要启用本地 TorchScript RL policy，可切换为 `motion_mode:=rl`。RL 模式需要容器内安装 PyTorch。

## 第三方源码

如果你拿到的是完整项目压缩包或完整 git 工作区，应该已经包含下面这些目录，直接进入 Docker 编译即可，不需要重新下载。

| 路径 | 来源 | 当前用途 | 获取方式 |
| --- | --- | --- | --- |
| `third_party/tare_planner` | TARE | TARE 探索主算法 | `git clone https://github.com/caochao39/tare_planner.git third_party/tare_planner` |
| `third_party/HPHS` | HPHS | HPHS 探索算法、`move_base`、`octomap_server` 配置 | `git clone https://github.com/bit-lsj/HPHS.git third_party/HPHS` |
| `third_party/autonomous_exploration_development_environment` | CMU exploration development environment | `local_planner`、`pathFollower`、`terrain_analysis`、`sensor_scan_generation`、Velodyne 仿真包等 | `git clone https://github.com/HongbiaoZ/autonomous_exploration_development_environment.git third_party/autonomous_exploration_development_environment` |
| `third_party/unitree_ros-master` | Unitree ROS | A1 URDF、Gazebo、关节控制器、Building world | `git clone https://github.com/unitreerobotics/unitree_ros.git third_party/unitree_ros-master` |
| `third_party/unitree_guide_upstream` | Unitree guide | `unitree_guide/launch/gazeboSim.launch` 和 `unitree_move_base` 参考 | `git clone https://github.com/unitreerobotics/unitree_guide.git third_party/unitree_guide_upstream` |
| `third_party/unitree_ros_to_real_upstream` | Unitree ROS-to-real | `unitree_legged_msgs` 消息定义 | `git clone https://github.com/unitreerobotics/unitree_ros_to_real.git third_party/unitree_ros_to_real_upstream` |
| `third_party/3d-navi` | 3d-navi | 参考资料，不参与当前 catkin 编译 | `git clone https://gitee.com/fdsf3e2342/3d-navi.git third_party/3d-navi` |
| `third_party/rl_policy` | 本地 TorchScript RL policy | 可选 A1 真实 policy 运动后端，默认站立模式不依赖它 | 随完整工作区分发；缺失时不影响 `motion_mode:=standing` |

本工作区实测过的上游 commit：

```text
TARE:        caochao39/tare_planner                         4450059
HPHS:        bit-lsj/HPHS                                   62914f8
CMU stack:   HongbiaoZ/autonomous_exploration_development_environment bf0cba7
Unitree guide: unitreerobotics/unitree_guide                fdf4d23
Unitree msgs:  unitreerobotics/unitree_ros_to_real          b989870
```

重新获取第三方源码的示例：

```bash
mkdir -p third_party

git clone https://github.com/caochao39/tare_planner.git third_party/tare_planner
git clone https://github.com/bit-lsj/HPHS.git third_party/HPHS
git clone https://github.com/HongbiaoZ/autonomous_exploration_development_environment.git third_party/autonomous_exploration_development_environment
git clone https://github.com/unitreerobotics/unitree_ros.git third_party/unitree_ros-master
git clone https://github.com/unitreerobotics/unitree_guide.git third_party/unitree_guide_upstream
git clone https://github.com/unitreerobotics/unitree_ros_to_real.git third_party/unitree_ros_to_real_upstream
git clone https://gitee.com/fdsf3e2342/3d-navi.git third_party/3d-navi

git -C third_party/tare_planner checkout 4450059
git -C third_party/HPHS checkout 62914f8
git -C third_party/autonomous_exploration_development_environment checkout bf0cba7
git -C third_party/unitree_guide_upstream checkout fdf4d23
git -C third_party/unitree_ros_to_real_upstream checkout b989870
```

注意：本项目已经对部分第三方源码做了 Noetic/A1 集成修改，例如 HPHS Python 兼容、costmap 参数、CMU 静态 TF、Unitree Building 场景等。重新 clone 上游仓库后，需要重新应用本项目的修改，或者直接使用本工作区随附的 `third_party/` 版本。

## Catkin 工作区入口

`src/vendor/` 里的目录是软链接，用来把第三方包挂进当前 catkin 工作区。别人重新 clone 第三方源码后，如果 `src/vendor` 不存在，可以这样重建：

```bash
mkdir -p src/vendor

ln -sfn ../../third_party/HPHS src/vendor/HPHS
ln -sfn ../../third_party/tare_planner/src/tare_planner src/vendor/tare_planner

ln -sfn ../../third_party/autonomous_exploration_development_environment/src/joystick_drivers src/vendor/joystick_drivers
ln -sfn ../../third_party/autonomous_exploration_development_environment/src/loam_interface src/vendor/loam_interface
ln -sfn ../../third_party/autonomous_exploration_development_environment/src/local_planner src/vendor/local_planner
ln -sfn ../../third_party/autonomous_exploration_development_environment/src/sensor_scan_generation src/vendor/sensor_scan_generation
ln -sfn ../../third_party/autonomous_exploration_development_environment/src/terrain_analysis src/vendor/terrain_analysis
ln -sfn ../../third_party/autonomous_exploration_development_environment/src/terrain_analysis_ext src/vendor/terrain_analysis_ext
ln -sfn ../../third_party/autonomous_exploration_development_environment/src/vehicle_simulator src/vendor/vehicle_simulator
ln -sfn ../../third_party/autonomous_exploration_development_environment/src/velodyne_simulator src/vendor/velodyne_simulator
ln -sfn ../../third_party/autonomous_exploration_development_environment/src/visualization_tools src/vendor/visualization_tools
ln -sfn ../../third_party/autonomous_exploration_development_environment/src/waypoint_example src/vendor/waypoint_example
ln -sfn ../../third_party/autonomous_exploration_development_environment/src/waypoint_rviz_plugin src/vendor/waypoint_rviz_plugin

ln -sfn ../../third_party/unitree_ros-master/robots/a1_description src/vendor/a1_description
ln -sfn ../../third_party/unitree_ros-master/unitree_controller src/vendor/unitree_controller
ln -sfn ../../third_party/unitree_ros-master/unitree_gazebo src/vendor/unitree_gazebo
ln -sfn ../../third_party/unitree_ros-master/unitree_legged_control src/vendor/unitree_legged_control
ln -sfn ../../third_party/unitree_guide_upstream/unitree_guide src/vendor/unitree_guide
ln -sfn ../../third_party/unitree_guide_upstream/unitree_move_base src/vendor/unitree_move_base
ln -sfn ../../third_party/unitree_ros_to_real_upstream/unitree_legged_msgs src/vendor/unitree_legged_msgs
```

确认软链接是否正确：

```bash
find src/vendor -maxdepth 1 -type l -printf '%f -> %l\n' | sort
```

## 系统依赖安装

在 Docker 容器内安装依赖：

```bash
apt-get update
apt-get install -y \
  git \
  build-essential \
  cmake \
  python3-numpy \
  python3-opencv \
  joystick \
  libgoogle-glog-dev \
  libqt5core5a \
  libqt5gui5 \
  libqt5widgets5 \
  qtbase5-dev \
  ros-noetic-controller-manager \
  ros-noetic-effort-controllers \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-ros-control \
  ros-noetic-joint-state-controller \
  ros-noetic-move-base \
  ros-noetic-move-base-msgs \
  ros-noetic-octomap-server \
  ros-noetic-pcl-ros \
  ros-noetic-robot-state-publisher \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-rviz \
  ros-noetic-tf2-sensor-msgs \
  ros-noetic-xacro
```

如果使用 `osrf/ros:noetic-desktop-full`，其中很多 ROS 包已经存在；重复安装不会有问题。当前 HPHS 已移除对 `pyquaternion` 的依赖，不需要 `pip install pyquaternion`。

`joystick_drivers/ps3joy` 是可选手柄支持，当前自动探索仿真不依赖它，并通过 `CATKIN_IGNORE` 跳过。如果要恢复 PS3 手柄节点，需要额外安装 `libusb-dev`、`bluez`、`python3-bluez` 并删除 `third_party/autonomous_exploration_development_environment/src/joystick_drivers/ps3joy/CATKIN_IGNORE`。

## Quick Start：完整跑起来

下面是从当前机器状态完整跑起来项目的最短路径。推荐每次切换 TARE / HPHS 前先重启容器，避免上一次 Gazebo 或 ROS 节点残留。

### 1. 启动或创建 Docker

可以使用本机已有容器，也可以直接拉取已配置好的 DockerHub 环境镜像。镜像只包含 ROS / Gazebo / PyTorch 等运行环境，不包含 bind mount 的项目源码；仍需要把项目父目录挂载到 `/workspace`。

DockerHub 镜像地址占位：

```text
<dockerhub_namespace>/3dnav-ros-noetic:base
<dockerhub_namespace>/3dnav-ros-noetic:x11
```

如果 `ros-noetic` 容器已经存在：

```bash
docker start ros-noetic
docker exec -it ros-noetic bash
```

如果容器不存在，先创建：

```bash
docker run -it --name ros-noetic --net=host \
  -v /home/wya/nav:/workspace \
  osrf/ros:noetic-desktop-full bash
```

如果使用已配置好的 DockerHub 镜像创建：

```bash
docker pull <dockerhub_namespace>/3dnav-ros-noetic:base
docker run -it --name ros-noetic --net=host \
  -v /home/wya/nav:/workspace \
  <dockerhub_namespace>/3dnav-ros-noetic:base bash
```

这里 `/home/wya/nav` 是宿主机上的项目父目录。别人复现时把它替换成自己的路径即可，例如项目位于 `/home/alice/nav/3d_nav`，就挂载：

```bash
docker run -it --name ros-noetic --net=host \
  -v /home/alice/nav:/workspace \
  osrf/ros:noetic-desktop-full bash
```

容器内保持项目路径为 `/workspace/3d_nav`，后续命令可以原样使用。

### 1.1 可视化 Docker：Gazebo / RViz

普通 `ros-noetic` 容器适合 headless 运行。如果要在 Docker 里打开 Gazebo GUI 或 RViz，使用带 X11 挂载的 `ros-noetic-x11` 容器。

首次创建 X11 容器：

```bash
xhost +si:localuser:root
docker commit ros-noetic ros-noetic-3dnav:x11-base
docker run -dit --name ros-noetic-x11 --net=host \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/wya/nav:/workspace \
  ros-noetic-3dnav:x11-base bash
```

如果使用已配置好的 DockerHub X11 镜像创建：

```bash
docker pull <dockerhub_namespace>/3dnav-ros-noetic:x11
xhost +si:localuser:root
docker run -dit --name ros-noetic-x11 --net=host \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/wya/nav:/workspace \
  <dockerhub_namespace>/3dnav-ros-noetic:x11 bash
docker exec -it ros-noetic-x11 bash
```

这里也要把 `/home/wya/nav` 替换为自己的项目父目录。

本机已经创建并测试通过 `ros-noetic-x11`，正常使用时执行：

```bash
docker start ros-noetic-x11
docker exec -it ros-noetic-x11 bash
```

容器内测试 X11 / Qt：

```bash
source /opt/ros/noetic/setup.bash
python3 - <<'PY'
from python_qt_binding import QtWidgets
app = QtWidgets.QApplication([])
geo = app.primaryScreen().geometry()
print("QT_OK", geo.width(), geo.height())
app.quit()
PY
```

正常会输出类似：

```text
QT_OK 2560 1440
```

### 1.2 给 Docker 补 pip 和 PyTorch

`osrf/ros:noetic-desktop-full` 默认没有 `pip` 和 PyTorch；当前网络下 Ubuntu apt 源也可能不可用，所以本项目使用宿主机下载 wheel、容器内离线安装的方式。如果使用上面的 DockerHub 镜像，PyTorch 已经内置，通常不需要执行本节安装脚本。

当前机器已经准备好本地缓存：

```text
/home/wya/nav/3d_nav/.cache/torch_wheels
```

如果缓存缺失，在宿主机执行：

```bash
cd /home/wya/nav/3d_nav
mkdir -p .cache/torch_wheels
python3 -m pip download --dest .cache/torch_wheels \
  --only-binary=:all: \
  --platform linux_x86_64 \
  --python-version 38 \
  --implementation cp \
  --abi cp38 \
  --index-url https://download.pytorch.org/whl/cpu \
  --extra-index-url https://pypi.org/simple \
  "torch==2.4.1+cpu" \
  "pip==24.3.1"
```

然后分别给普通容器和 X11 容器安装：

```bash
docker exec -it ros-noetic bash -lc \
  'cd /workspace/3d_nav && ./scripts/install_torch_offline.sh'

docker exec -it ros-noetic-x11 bash -lc \
  'cd /workspace/3d_nav && ./scripts/install_torch_offline.sh'
```

正常输出应包含：

```text
pip 24.3.1 ... (python 3.8)
TORCH_OK 2.4.1+cpu
POLICY_OK (1, 12)
```

注意不要直接 `pip install .cache/torch_wheels/*.whl`，缓存里可能有给宿主机准备的 wheel；例如较新的 `setuptools` 不一定兼容 ROS Noetic 的 Python 3.8。脚本只安装 RL policy 运行必需的 `pip` 和 `torch`。

如果不再需要允许 root 访问 X11，可以在宿主机执行：

```bash
xhost -si:localuser:root
```

### 2. 进入工作区并编译

在容器内执行：

```bash
cd /workspace/3d_nav
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 3. 运行 TARE 版本

在容器内执行：

```bash
cd /workspace/3d_nav
source devel/setup.bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=standing \
  use_standing_driver:=true stand_height:=0.34
```

如果需要 Gazebo GUI 和 RViz：

```bash
docker exec -it ros-noetic-x11 bash
cd /workspace/3d_nav
source devel/setup.bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=true headless:=false paused:=false rviz:=true \
  motion_mode:=standing \
  use_standing_driver:=true stand_height:=0.34
```

### 4. 运行 HPHS 版本

切换到 HPHS 前，建议先在宿主机执行：

```bash
docker restart ros-noetic
docker exec -it ros-noetic bash
```

如果使用可视化容器，则执行：

```bash
docker restart ros-noetic-x11
docker exec -it ros-noetic-x11 bash
```

然后在容器内执行：

```bash
cd /workspace/3d_nav
source devel/setup.bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=standing \
  use_standing_driver:=true stand_height:=0.34 \
  tf_time_offset:=0.10
```

如果需要 Gazebo GUI 和 RViz：

```bash
docker exec -it ros-noetic-x11 bash
cd /workspace/3d_nav
source devel/setup.bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=true headless:=false paused:=false rviz:=true \
  motion_mode:=standing \
  use_standing_driver:=true stand_height:=0.34 \
  tf_time_offset:=0.10
```

### 5. 验证是否跑起来

另开一个终端进入容器：

```bash
docker exec -it ros-noetic bash
cd /workspace/3d_nav
source devel/setup.bash
```

检查关键节点：

```bash
rosnode list
```

检查关键话题频率：

```bash
rostopic hz /registered_scan
rostopic hz /scan
rostopic hz /terrain_map
rostopic hz /sensor_scan
```

检查探索目标和速度输出：

```bash
rostopic echo -n 1 /way_point
rostopic echo -n 1 /cmd_vel
rostopic echo -n 1 /a1_gazebo/FL_thigh_controller/command
```

正常情况下可以看到：

```text
/registered_scan  约 10 Hz
/scan             约 10 Hz
/terrain_map      约 9-10 Hz
/sensor_scan      约 10 Hz
/way_point        有输出
/cmd_vel          有输出
/a1_gazebo/FL_thigh_controller/command 有输出
```

`rosnode list` 中应包含 `/standing_a1_driver`。

当前默认规划线速度已降为：

```text
planner_max_speed: 1.0 m/s
planner_autonomy_speed: 1.0 m/s
max_linear: 1.0 m/s
max_angular: 1.5 rad/s
```

其中 `/cmu_cmd_vel` 是 local_planner 输出，`/cmd_vel` 是 `cmu_a1_bridge` 限幅后发给 A1 standing / RL driver 的速度命令。

### 修改速度限制

速度链路分三层：

```text
local_planner/pathFollower  ->  /cmu_cmd_vel
cmu_a1_bridge               ->  /cmd_vel
A1 standing / RL driver     ->  Gazebo A1
```

当前默认值：

```text
规划线速度:       planner_max_speed:=1.0
自主巡航线速度:   planner_autonomy_speed:=1.0
bridge 线速度限幅: max_linear:=1.0
bridge 角速度限幅: max_angular:=1.5
```

临时修改 TARE 速度限制：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=standing \
  planner_max_speed:=1.0 \
  planner_autonomy_speed:=1.0 \
  max_linear:=1.0 \
  max_angular:=1.5
```

临时修改 HPHS 速度限制：

```bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=standing \
  planner_max_speed:=1.0 \
  planner_autonomy_speed:=1.0 \
  max_linear:=1.0 \
  max_angular:=1.5
```

如果只单独启动 bridge，可以直接改：

```bash
roslaunch a1_exploration_bridge bridge.launch \
  max_linear:=1.0 \
  max_angular:=1.5
```

永久修改默认值时看这些位置：

```text
src/a1_exploration_bridge/launch/tare_a1_building.launch
src/a1_exploration_bridge/launch/hphs_a1_building.launch
src/a1_exploration_bridge/launch/bridge.launch
src/a1_exploration_bridge/scripts/cmu_a1_bridge.py
```

修改 launch 参数后重新启动 launch 即可；修改 Python 脚本默认值后也只需要重启对应 ROS 节点，不需要重新 `catkin_make`。

运行时确认实际速度：

```bash
rostopic echo -n 1 /cmu_cmd_vel
rostopic echo -n 1 /cmd_vel
```

`/cmu_cmd_vel` 是规划器原始输出；`/cmd_vel` 是经过 `cmu_a1_bridge` 限幅后的实际 A1 速度命令。

HPHS 还可以额外检查：

```bash
rostopic hz /map
rostopic info /scan
```

其中 `/scan` 应该显示：

```text
Publishers:
 * /cmu_a1_bridge

Subscribers:
 * /move_base
```

### 6. 停止和清理

在运行 launch 的终端按 `Ctrl-C` 停止。如果停止后仍有 Gazebo / ROS 残留，直接重启容器：

```bash
docker restart ros-noetic
```

也可以不重启容器，在容器内清理 ROS/Gazebo 运行态：

```bash
cd /workspace/3d_nav
./scripts/cleanup_ros_gazebo.sh
```

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

## A1 运动模式

本工作区支持两种 A1 Gazebo 运动后端，由 `motion_mode` 选择：

| 模式 | 参数 | 节点 | 用途 |
| --- | --- | --- | --- |
| 固定站姿移动 | `motion_mode:=standing` | `standing_a1_driver.py` | 默认模式，不需要 policy / PyTorch，稳定跑通 TARE / HPHS / 雷达 / 地图 / 可视化 |
| RL policy 驱动 | `motion_mode:=rl` | `a1_rl_policy_driver.py` | 使用本地 TorchScript policy 控制 12 个关节，依赖 PyTorch |

默认仍是 `motion_mode:=standing`，所以原有 TARE / HPHS 运行方式不受影响。

### 固定站姿模式

为了先完整验证 TARE / HPHS / 雷达 / 地图 / 控制链路，本工作区新增临时节点：

```text
src/a1_exploration_bridge/scripts/standing_a1_driver.py
```

这个节点做两件事：

- 持续给 12 个腿部关节发布固定目标，让 A1 锁在类似 Unitree guide `FixedStand` 的站立姿态。
- 订阅 `/cmd_vel`，把速度命令积分成 Gazebo 里 `a1_gazebo` model 的位置变化，让机器人以站立姿态移动。

默认站立姿态：

| 关节 | 目标角度 rad | Kp | Kd |
| --- | ---: | ---: | ---: |
| hip | `0.0` | `180` | `8` |
| thigh | `0.67` | `180` | `8` |
| calf | `-1.3` | `300` | `15` |

默认站立高度：

```text
stand_height:=0.34
```

这个节点在下面三个 launch 中默认开启：

```text
a1_exploration_bridge/a1_building_sim.launch
a1_exploration_bridge/tare_a1_building.launch
a1_exploration_bridge/hphs_a1_building.launch
```

固定站姿模式只启动 `standing_a1_driver.py`，不会启动 `a1_rl_policy_driver.py`，也不会读取 `rl_policy_path`。启动 standing 时不要传 `rl_policy_path`；只有 `motion_mode:=rl` 才需要 policy 文件和 PyTorch。

正常运行时可以显式保留：

```bash
use_standing_driver:=true stand_height:=0.34
```

如果不想启动任何 A1 运动后端，可以关闭临时站立驱动：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  use_standing_driver:=false
```

注意：这不是物理真实的四足行走。它绕过足端接触和步态控制，只适合在缺少 walking policy 时临时跑通探索、建图、避障、可视化和 ROS topic 链路。

### RL policy 模式

本地 policy 文件：

```text
third_party/rl_policy/a1/policy.pt
```

该文件已检查为 TorchScript `Sequential` policy：

```text
input:  45
output: 12
```

RL 驱动节点：

```text
src/a1_exploration_bridge/scripts/a1_rl_policy_driver.py
```

默认 observation 结构：

```text
ang_vel(3) + projected_gravity(3) + command(3) + dof_pos_error(12) + dof_vel(12) + last_action(12)
```

其中 12 维关节/action 顺序必须和 policy 训练时一致，当前按腿优先排列：

```text
FL_hip_joint, FL_thigh_joint, FL_calf_joint,
FR_hip_joint, FR_thigh_joint, FR_calf_joint,
RL_hip_joint, RL_thigh_joint, RL_calf_joint,
RR_hip_joint, RR_thigh_joint, RR_calf_joint
```

如果顺序错成按关节类型分组，或者把 `FR` / `RL` 对调，policy 输出会发布到错误腿和错误关节，仿真释放后很容易侧翻。

训练参数已按当前 policy 配置：

```text
control_type: P
Kp: 30.0
Kd: 1.0
action_scale: 0.25
ang_vel scale: 0.25
dof_pos scale: 1.0
dof_vel scale: 0.05
command scale: [1.0, 1.0, 1.0]
```

默认关节角度：

| 关节顺序 | 默认角 rad |
| --- | ---: |
| `FL_hip_joint` | `0.0` |
| `FL_thigh_joint` | `0.75` |
| `FL_calf_joint` | `-1.5` |
| `FR_hip_joint` | `0.0` |
| `FR_thigh_joint` | `0.75` |
| `FR_calf_joint` | `-1.5` |
| `RL_hip_joint` | `0.0` |
| `RL_thigh_joint` | `0.75` |
| `RL_calf_joint` | `-1.5` |
| `RR_hip_joint` | `0.0` |
| `RR_thigh_joint` | `0.75` |
| `RR_calf_joint` | `-1.5` |

ROS Noetic Docker 里默认没有 `pip` 和 PyTorch。启用 RL 前先在容器中确认：

```bash
python3 - <<'PY'
import torch
print(torch.__version__)
PY
```

如果没有 `torch`，使用前文“给 Docker 补 pip 和 PyTorch”的离线方式安装：

```bash
cd /workspace/3d_nav
./scripts/install_torch_offline.sh
```

当前 `ros-noetic` 和 `ros-noetic-x11` 容器已经验证通过：

```text
pip 24.3.1
TORCH_OK 2.4.1+cpu
POLICY_OK (1, 12)
```

也可以手动检查 policy：

```bash
cd /workspace/3d_nav
python3 - <<'PY'
import torch
p = "third_party/rl_policy/a1/policy.pt"
m = torch.jit.load(p, map_location="cpu")
y = m(torch.zeros(1, 45))
print("POLICY_OK", tuple(y.shape))
PY
```

正常输出：

```text
POLICY_OK (1, 12)
```

启用 RL policy：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=rl \
  rl_policy_path:=/workspace/3d_nav/third_party/rl_policy/a1/policy.pt
```

HPHS 同理：

```bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=rl \
  rl_policy_path:=/workspace/3d_nav/third_party/rl_policy/a1/policy.pt \
  tf_time_offset:=0.10
```

RL 模式不再使用 `standing_a1_driver.py` 的 Gazebo model pose 平移，而是由 policy 输出 12 个关节目标驱动 Unitree Gazebo 关节控制器。是否能稳定行走取决于 policy 的训练环境、观测定义、仿真物理参数和启动姿态是否匹配。

## Docker 环境

宿主机是 Ubuntu 22.04 时，推荐用 Docker 运行 ROS1 Noetic，避免把 Ubuntu 20.04 的 ROS apt 源混入宿主机。

首次拉取 ROS Noetic 镜像：

```bash
docker pull osrf/ros:noetic-desktop-full
```

创建并进入容器：

```bash
docker run -it --name ros-noetic --net=host \
  -v /home/wya/nav:/workspace \
  osrf/ros:noetic-desktop-full bash
```

如果容器已存在，直接启动并进入：

```bash
docker start ros-noetic
docker exec -it ros-noetic bash
```

进入容器后初始化 ROS 环境：

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/3d_nav
```

首次使用时按前文“系统依赖安装”章节安装完整 apt 依赖。最少需要 `move_base`、`octomap_server`、`tf2_sensor_msgs`、`pcl_ros`、Unitree 控制器相关 ROS 包、`libgoogle-glog-dev`、`python3-opencv` 等。

如果宿主机 Docker 需要权限，把上述宿主机上的 `docker ...` 命令改成 `sudo docker ...`。

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
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=standing \
  use_standing_driver:=true stand_height:=0.34
```

打开 GUI / RViz 时可改为：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=true headless:=false paused:=false rviz:=true \
  motion_mode:=standing \
  use_standing_driver:=true stand_height:=0.34
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
  /standing_a1_driver

关键话题：
  /registered_scan  约 10 Hz
  /scan             约 10 Hz
  /terrain_map      约 9-10 Hz
  /sensor_scan      约 10 Hz
  /way_point        约 1 Hz
  /cmd_vel          约 50 Hz
  /a1_gazebo/FL_thigh_controller/command 约 50 Hz

样例：
  /way_point frame_id = map
  /cmd_vel angular.z <= 1.5
  /a1_gazebo/FL_thigh_controller/command q = 0.67
```

## 运行 HPHS

容器内执行：

```bash
cd /workspace/3d_nav
source devel/setup.bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=standing \
  use_standing_driver:=true stand_height:=0.34 \
  tf_time_offset:=0.10
```

打开 GUI / RViz 时可改为：

```bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=true headless:=false paused:=false rviz:=true \
  motion_mode:=standing \
  use_standing_driver:=true stand_height:=0.34 \
  tf_time_offset:=0.10
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
  /standing_a1_driver

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
  /a1_gazebo/FL_thigh_controller/command 约 50 Hz

样例：
  /way_point frame_id = map
  /cmu_cmd_vel frame_id = vehicle
  /cmd_vel angular.z <= 1.5
  /a1_gazebo/FL_thigh_controller/command q = 0.67
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

### 雷达安装位置和高度

A1 模型里的 VLP-16 安装在 `trunk` 上，URDF 位置在：

```text
third_party/unitree_ros-master/robots/a1_description/xacro/robot.xacro
```

当前外参为：

```text
xyz="0.12 0 0.19"
rpy="0 0 0"
```

运行 TARE / HPHS 时，不需要分别去改 TARE 或 HPHS 算法内部参数。统一设置位置在桥接 launch：

```text
src/a1_exploration_bridge/launch/bridge.launch
```

对应参数是：

```text
scan_offset_x:=0.12
scan_offset_y:=0.0
scan_offset_z:=0.19
```

这三个值表示雷达相对 A1 机体局部坐标原点的安装偏移，单位是米：`x` 向前，`y` 向左，`z` 向上。`cmu_a1_bridge.py` 会用它们把 `/velodyne_points` 注册到 `/registered_scan`，并生成 HPHS `move_base` 使用的 `/scan`。

为了方便一键启动时覆盖，TARE 和 HPHS 顶层 launch 也暴露了同名参数：

```text
src/a1_exploration_bridge/launch/tare_a1_building.launch
src/a1_exploration_bridge/launch/hphs_a1_building.launch
```

示例：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  scan_offset_x:=0.12 scan_offset_y:=0.0 scan_offset_z:=0.19

roslaunch a1_exploration_bridge hphs_a1_building.launch \
  scan_offset_x:=0.12 scan_offset_y:=0.0 scan_offset_z:=0.19
```

如果使用当前临时站立驱动，A1 机体高度由 `stand_height` 控制，默认：

```text
stand_height:=0.34
```

因此默认雷达中心离地高度约为：

```text
stand_height + scan_offset_z = 0.34 + 0.19 = 0.53 m
```

注意：HPHS 上游旧 launch 里的 `vehicleHeight:=0.75` 属于旧小车仿真器 `cmu_vehicle_simulator`，当前 A1 版本没有启动它，不要用这个值来设置 A1 的雷达安装高度。

常用参数：

```bash
roslaunch a1_exploration_bridge bridge.launch \
  state_source:=gazebo_model_states \
  model_name:=a1_gazebo \
  scan_in:=/velodyne_points \
  registered_scan_out:=/registered_scan \
  laserscan_out:=/scan \
  transform_scan_to_map:=true \
  tf_time_offset:=0.02 \
  scan_offset_x:=0.12 \
  scan_offset_y:=0.0 \
  scan_offset_z:=0.19 \
  max_linear:=1.0 \
  max_angular:=1.5
```

## 验收命令

构建检查：

```bash
cd /workspace/3d_nav
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -m py_compile \
  src/a1_exploration_bridge/scripts/a1_rl_policy_driver.py \
  src/a1_exploration_bridge/scripts/cmu_a1_bridge.py \
  src/a1_exploration_bridge/scripts/standing_a1_driver.py \
  third_party/HPHS/scripts/explorer.py
```

RL policy 文件形状检查：

```bash
python3 - <<'PY'
import torch
m = torch.jit.load("third_party/rl_policy/a1/policy.pt", map_location="cpu")
y = m(torch.zeros(1, 45))
assert tuple(y.shape) == (1, 12), tuple(y.shape)
print("POLICY_OK", tuple(y.shape))
PY
```

RL 运行时检查：

```bash
roslaunch a1_exploration_bridge a1_building_sim.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=rl \
  rl_policy_path:=/workspace/3d_nav/third_party/rl_policy/a1/policy.pt
```

另开一个终端进入同一容器：

```bash
cd /workspace/3d_nav
source devel/setup.bash
rosnode list | grep a1_rl_policy_driver
rostopic echo -n 1 /a1_gazebo/FL_thigh_controller/command
```

正常会看到 `/a1_rl_policy_driver`，并且关节命令里有类似 `Kp: 30.0`、`Kd: 1.0`、`q: 0.75` 附近的输出。

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
rostopic hz /a1_gazebo/FL_thigh_controller/command
rostopic echo -n 1 /a1_gazebo/FL_thigh_controller/command
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

如果上一次 launch 没有正常退出，可能出现 Gazebo entity 已存在、ROS 节点重名或端口状态残留。典型报错：

```text
Spawn status: SpawnModel: Failure - entity already exists.
Spawn service failed. Exiting.
Failed to load joint_state_controller
ServiceException: service [/a1_gazebo/controller_manager/load_controller] returned no response
```

原因是 Gazebo 里已经有一个叫 `a1_gazebo` 的模型，或者旧的 `gzserver` / `rosmaster` 仍在运行。优先在容器内执行：

```bash
cd /workspace/3d_nav
./scripts/cleanup_ros_gazebo.sh
source devel/setup.bash
```

也可以直接重启容器：

```bash
docker restart ros-noetic
docker exec -it ros-noetic bash
cd /workspace/3d_nav
source devel/setup.bash
```

### `spawn_model` / `controller_manager` 启动失败

如果看到类似下面的报错：

```text
rospy.exceptions.ROSInitException: init_node interrupted before it could complete
Failed to load joint_state_controller
ServiceException: service [/a1_gazebo/controller_manager/load_controller] returned no response
```

通常不是编译错误，而是上一次 Gazebo/ROS 没有完整退出，留下了 `gzserver`、`cmu_a1_bridge` 或半残留 ROS graph。先在宿主机清理容器：

```bash
docker restart ros-noetic
```

然后重新进入容器运行：

```bash
docker exec -it ros-noetic bash
cd /workspace/3d_nav
source devel/setup.bash
```

再启动 TARE 或 HPHS：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false
```

或：

```bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false
```

确认残留进程是否已清理：

```bash
pgrep -af "roslaunch|rosmaster|gzserver|gzclient|spawn_model|controller_manager|cmu_a1_bridge"
```

如果只看到 `pgrep` 自己这一行，说明环境已经干净。

### HPHS 有 TF extrapolation 日志

HPHS 的 `move_base` 在仿真时间下如果看到类似：

```text
Lookup would require extrapolation ... from frame [vehicle] to frame [map]
Could not transform the global plan to the frame of the controller
```

这是 `map -> sensor -> vehicle` TF 链和 `move_base` 控制周期之间的毫秒级时间戳差异。本工作区已做两处处理：

- `sensor -> vehicle` 和 `sensor -> camera` 使用 `tf2_ros/static_transform_publisher` 发布到 `/tf_static`，避免老版 `tf/static_transform_publisher` 以 10Hz 时间戳刷新静态 TF。
- HPHS 的 `map -> sensor` 动态 TF 默认使用更大的 future dating。

桥接节点通用默认值是：

```text
tf_time_offset:=0.02
```

HPHS 当前默认和推荐值是：

```bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  tf_time_offset:=0.10
```

如果 `/map`、`/way_point`、`/cmu_cmd_vel`、`/cmd_vel` 都持续发布，说明整体链路仍在运行。

### Gazebo Fuel SSL 报错

如果看到：

```text
libcurl: (35) OpenSSL SSL_connect: SSL_ERROR_SYSCALL in connection to fuel.ignitionrobotics.org:443
```

这是 Gazebo Classic 尝试访问在线 Fuel 模型库失败。本项目使用本地 A1 和 Building 模型，不依赖在线模型库；`a1_building_sim.launch` 已设置：

```text
GAZEBO_MODEL_DATABASE_URI=""
```

重新启动 launch 后该在线访问会被禁用。如果仍看到一次旧日志，通常是上一次 Gazebo 进程残留，重启容器即可：

```bash
docker restart ros-noetic
```

### HPHS costmap missed rate

HPHS 的 `move_base` costmap 在本工作区中已从上游默认 10Hz 降到 1Hz，以减轻 Gazebo + octomap + costmap 同时运行时的 CPU 压力。如果仍偶尔看到：

```text
Map update loop missed its desired rate
```

这是性能 warning，不是启动失败。优先确认下面这些话题是否仍在发布：

```bash
rostopic hz /map
rostopic echo -n 1 /way_point
rostopic echo -n 1 /cmd_vel
```

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
