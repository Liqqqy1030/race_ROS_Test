cmake_minimum_required(VERSION 2.8.3)
project(abot_patrol_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  tf
)

catkin_package()

catkin_install_python(
  PROGRAMS scripts/patrol.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
