# 3d_nav: TARE / HPHS on Unitree A1
## 1. 选择容器

本项目可以使用本机已有容器，也可以直接拉取已配置好的 DockerHub 环境镜像。镜像只包含 ROS / Gazebo / PyTorch 等运行环境，不包含 bind mount 的项目源码；仍需要把项目父目录挂载到 `/workspace`。

DockerHub 镜像地址占位：

```text
<dockerhub_namespace>/3dnav-ros-noetic:base
<dockerhub_namespace>/3dnav-ros-noetic:x11
```

无可视化 / headless：

```bash
docker start ros-noetic
docker exec -it ros-noetic bash
```

如果本机没有 `ros-noetic` 容器，可以从 DockerHub 镜像创建：

```bash
docker pull <dockerhub_namespace>/3dnav-ros-noetic:base
docker run -it --name ros-noetic --net=host \
  -v /home/wya/nav:/workspace \
  <dockerhub_namespace>/3dnav-ros-noetic:base bash
```

有可视化 / Gazebo GUI + RViz：

```bash
xhost +si:localuser:root
docker start ros-noetic-x11
docker exec -it ros-noetic-x11 bash
```

如果本机没有 `ros-noetic-x11` 容器，可以从 DockerHub 镜像创建：

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

进入容器后统一执行：

```bash
cd /workspace/3d_nav
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

固定站姿模式 `motion_mode:=standing` 不需要 policy，也不需要 PyTorch。

RL 模式 `motion_mode:=rl` 才需要 PyTorch。DockerHub 镜像已内置 PyTorch；如果使用的是原始 `osrf/ros:noetic-desktop-full` 容器，且容器里还没有 `torch`：

```bash
cd /workspace/3d_nav
./scripts/install_torch_offline.sh
source devel/setup.bash
```

启动前如果担心有残留：

```bash
cd /workspace/3d_nav
./scripts/cleanup_ros_gazebo.sh
source devel/setup.bash
```

## 2. TARE 启动

TARE + 固定站姿 + 无可视化：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=standing
```

这个模式只启动 `standing_a1_driver.py`，不启动 `a1_rl_policy_driver.py`，不要传 `rl_policy_path`。

TARE + 固定站姿 + 有可视化：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=true headless:=false paused:=false rviz:=true \
  motion_mode:=standing
```

TARE + RL policy + 无可视化：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=rl \
  rl_policy_path:=/workspace/3d_nav/third_party/rl_policy/a1/policy.pt
```

TARE + RL policy + 有可视化：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=true headless:=false paused:=false rviz:=true \
  motion_mode:=rl \
  rl_policy_path:=/workspace/3d_nav/third_party/rl_policy/a1/policy.pt
```

## 3. HPHS 启动

HPHS + 固定站姿 + 无可视化：

```bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=standing \
  tf_time_offset:=0.10
```

这个模式只启动 `standing_a1_driver.py`，不启动 `a1_rl_policy_driver.py`，不要传 `rl_policy_path`。

HPHS + 固定站姿 + 有可视化：

```bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=true headless:=false paused:=false rviz:=true \
  motion_mode:=standing \
  tf_time_offset:=0.10
```

HPHS + RL policy + 无可视化：

```bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=rl \
  rl_policy_path:=/workspace/3d_nav/third_party/rl_policy/a1/policy.pt \
  tf_time_offset:=0.10
```

HPHS + RL policy + 有可视化：

```bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=true headless:=false paused:=false rviz:=true \
  motion_mode:=rl \
  rl_policy_path:=/workspace/3d_nav/third_party/rl_policy/a1/policy.pt \
  tf_time_offset:=0.10
```

## 4. 常用参数

运动模式：

```text
motion_mode:=standing  固定站姿移动，不需要 policy / PyTorch，默认最稳定
motion_mode:=rl        TorchScript policy 驱动，依赖 PyTorch
```

速度限制：

```text
planner_max_speed:=1.0       local_planner 最大线速度
planner_autonomy_speed:=1.0  local_planner 自主巡航线速度
max_linear:=1.0              cmu_a1_bridge 输出 /cmd_vel 线速度限幅，m/s
max_angular:=1.5             cmu_a1_bridge 输出 /cmd_vel 角速度限幅，rad/s
```

示例：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=rl \
  planner_max_speed:=1.0 \
  planner_autonomy_speed:=1.0 \
  max_linear:=1.0 \
  max_angular:=1.5
```

## 5. 运行检查

另开一个终端进入同一容器：

```bash
cd /workspace/3d_nav
source devel/setup.bash
rosnode list
rostopic hz /registered_scan
rostopic hz /scan
rostopic echo -n 1 /way_point
rostopic echo -n 1 /cmu_cmd_vel
rostopic echo -n 1 /cmd_vel
rostopic echo -n 1 /a1_gazebo/FL_thigh_controller/command
```

RL policy 检查：

```bash
python3 - <<'PY'
import torch
m = torch.jit.load("third_party/rl_policy/a1/policy.pt", map_location="cpu")
y = m(torch.zeros(1, 45))
print("POLICY_OK", tuple(y.shape))
PY
```

正常输出：

```text
POLICY_OK (1, 12)
```

## 6. 常见问题

如果出现 `entity already exists`、`Failed to load joint_state_controller` 或 controller manager 无响应：

```bash
cd /workspace/3d_nav
./scripts/cleanup_ros_gazebo.sh
source devel/setup.bash
```

如果 X11 / RViz 打不开，确认使用的是 `ros-noetic-x11`，并在宿主机执行过：

```bash
xhost +si:localuser:root
```

如果 RL 模式提示没有 PyTorch：

```bash
cd /workspace/3d_nav
./scripts/install_torch_offline.sh
```
