# Porting Notes

This project replaces the CMU `vehicle_simulator` runtime vehicle with the 3d-navi Unitree A1 Gazebo platform while keeping the planner-side ROS interface stable.

## Interface Contract

The CMU exploration stack expects these topics:

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| A1/Gazebo -> planner | `/state_estimation` | `nav_msgs/Odometry` | High-rate vehicle pose and velocity in `map` |
| A1/Gazebo -> planner | `/registered_scan` | `sensor_msgs/PointCloud2` | Registered lidar cloud in `map` |
| planner -> A1 controller | `/cmu_cmd_vel` | `geometry_msgs/TwistStamped` | Output from CMU `local_planner` after remap |
| bridge -> A1 controller | `/cmd_vel` | `geometry_msgs/Twist` | Velocity command for A1 controller |

The bridge node can source odometry either from `/gazebo/model_states` or an existing odometry topic. Use `state_source:=odom odom_in:=/your_odom` when the A1 stack already publishes odometry.

## Launch Strategy

`tare_a1_building.launch` mirrors the TARE run sequence:

1. Start Unitree A1 in `Building.world`.
2. Start the bridge.
3. Start CMU `local_planner`, `terrain_analysis`, and `sensor_scan_generation`.
4. Start `tare_planner`.

`hphs_a1_building.launch` mirrors HPHS without launching `HPHS/launch/cmu_vehicle_simulator.launch`; that launch file is the part that spawns the old wheeled simulator.

## Expected Manual Step

Many Unitree A1 controller builds require a separate controller process such as `junior_ctrl` and a mode switch before `/cmd_vel` is accepted. Start that controller from the same sourced workspace if the dog spawns but ignores velocity commands.
