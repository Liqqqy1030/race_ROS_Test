cmake_minimum_required(VERSION 2.8.3)
project(competition_ai)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  TTS_audio
)

catkin_package(CATKIN_DEPENDS rospy std_msgs sensor_msgs TTS_audio)

catkin_install_python(PROGRAMS
  scripts/ros_ai_bridge.py
  scripts/ai_backend.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
