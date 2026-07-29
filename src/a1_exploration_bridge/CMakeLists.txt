cmake_minimum_required(VERSION 3.0.2)
project(a1_exploration_bridge)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cmu_a1_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
