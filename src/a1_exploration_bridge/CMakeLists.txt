cmake_minimum_required(VERSION 3.0.2)
project(a1_exploration_bridge)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  tf2_ros
  unitree_legged_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/a1_rl_policy_driver.py
  scripts/cmu_a1_bridge.py
  scripts/standing_a1_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
