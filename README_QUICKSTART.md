# 3d_nav: TARE / HPHS on Unitree A1
## 1. 选择容器

本项目可以使用本机已有容器，也可以从随项目分发的 Docker 镜像压缩包安装环境。镜像只包含 ROS / Gazebo / PyTorch 等运行环境，不包含 bind mount 的项目源码；仍需要把项目父目录挂载到 `/workspace`。

镜像压缩包路径：

```text
docker_img/3dnav-ros-noetic-base.tar.gz
docker_img/3dnav-ros-noetic-x11.tar.gz
```

如果本地没有 `docker_img/`，可以从百度网盘下载 Docker 镜像压缩包：

```text
链接: https://pan.baidu.com/s/1HXC63DsTsg0Vf5dRTuBoIA?pwd=fhj7
提取码: fhj7
```

无可视化 / headless：

```bash
docker start ros-noetic
docker exec -it ros-noetic bash
```

如果本机没有 `ros-noetic` 容器，可以先加载镜像包再创建：

```bash
docker load < docker_img/3dnav-ros-noetic-base.tar.gz
docker run -it --name ros-noetic --net=host \
  -v /home/wya/nav:/workspace \
  3dnav-ros-noetic:base bash
```

有可视化 / Gazebo GUI + RViz：

```bash
xhost +si:localuser:root
docker start ros-noetic-x11
docker exec -it ros-noetic-x11 bash
```

如果本机没有 `ros-noetic-x11` 容器，可以先加载镜像包再创建：

```bash
docker load < docker_img/3dnav-ros-noetic-x11.tar.gz
xhost +si:localuser:root
docker run -dit --name ros-noetic-x11 --net=host \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/wya/nav:/workspace \
  3dnav-ros-noetic:x11 bash
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

RL 模式 `motion_mode:=rl` 才需要 PyTorch。`docker_img` 镜像包已内置 PyTorch；如果使用的是原始 `osrf/ros:noetic-desktop-full` 容器，且容器里还没有 `torch`：

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
  rl_policy_path:=/workspace/3d_nav/third_party/rl_policy/a1/policy_act_inference_stair.pt
```

TARE + RL policy + 有可视化：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=true headless:=false paused:=false rviz:=true \
  motion_mode:=rl \
  rl_policy_path:=/workspace/3d_nav/third_party/rl_policy/a1/policy_act_inference_stair.pt
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
  rl_policy_path:=/workspace/3d_nav/third_party/rl_policy/a1/policy_act_inference_stair.pt \
  tf_time_offset:=0.10
```

HPHS + RL policy + 有可视化：

```bash
roslaunch a1_exploration_bridge hphs_a1_building.launch \
  gui:=true headless:=false paused:=false rviz:=true \
  motion_mode:=rl \
  rl_policy_path:=/workspace/3d_nav/third_party/rl_policy/a1/policy_act_inference_stair.pt \
  tf_time_offset:=0.10
```

## 4. 常用参数

运动模式：

```text
motion_mode:=rl        TorchScript policy 驱动，默认模式，依赖 PyTorch
motion_mode:=standing  固定站姿移动，不需要 policy / PyTorch，用于调试回退
```

速度限制：

```text
planner_max_speed:=1.0       local_planner 最大线速度
planner_autonomy_speed:=1.0  local_planner 自主巡航线速度
max_linear:=1.0              cmu_a1_bridge 输出 /cmd_vel 线速度限幅，m/s
max_angular:=1.5             cmu_a1_bridge 输出 /cmd_vel 角速度限幅，rad/s
```

A1 启动安全：

```text
spawn_z:=0.30                   A1 趴姿初始机身高度
spawn_joint_args:=...           spawn_model 初始 12 关节趴姿，默认已填好
rl_startup_damping_time:=5.0    初始 0 力矩 5 秒
rl_startup_damping_kp:=0.0      0 力矩阶段 Kp
rl_startup_damping_kd:=0.0      0 力矩阶段 Kd
rl_startup_prone_kp:=80.0       归位标准趴姿阶段 Kp
rl_startup_prone_kd:=1.0        归位标准趴姿阶段 Kd
rl_startup_prone_rate:=1.0      归位标准趴姿关节目标变化率限幅，rad/s
rl_startup_stand_kp:=80.0       慢站阶段 Kp
rl_startup_stand_kd:=1.0        慢站阶段 Kd
rl_startup_stand_rate:=0.75      慢站阶段关节目标变化率限幅，rad/s
startup_wait_for_policy:=true   bridge 等 RL driver 进入 policy
startup_cmd_hold_time:=2.0      policy active 后再保持 2 秒零速度
startup_cmd_ramp_time:=2.0      再用 2 秒把 planner 速度平滑放开
command_timeout:=0.5            planner 停止发布后 bridge 持续输出零速度
cmd_publish_rate:=20.0          bridge 定时发布 /cmd_vel，探索结束后也保持 RL 静止命令
head_mode:=none                 无头模式；改成 velocity 后使用速度方向 yaw PD
```

示例：

```bash
roslaunch a1_exploration_bridge tare_a1_building.launch \
  gui:=false headless:=true paused:=false rviz:=false \
  motion_mode:=rl \
  planner_max_speed:=1.0 \
  planner_autonomy_speed:=1.0 \
  max_linear:=1.0 \
  max_angular:=1.5 \
  head_mode:=velocity
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
m = torch.jit.load("third_party/rl_policy/a1/policy_act_inference_stair.pt", map_location="cpu")
y = m.act_inference(torch.zeros(1, 225))
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

如果 Gazebo GUI 里还显示上一次启动留下的运动轨迹，通常是 `gzclient` / `gzserver` / 轨迹可视化插件的历史显示没有清掉。先在当前容器清理：

```bash
cd /workspace/3d_nav
./scripts/cleanup_ros_gazebo.sh
source devel/setup.bash
```

如果使用 X11 容器，也在 `ros-noetic-x11` 里执行同样命令；最稳妥的方式是重启两个 host network 容器：

```bash
docker restart ros-noetic ros-noetic-x11
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
