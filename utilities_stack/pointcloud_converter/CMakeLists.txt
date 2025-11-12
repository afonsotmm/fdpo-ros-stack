cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_converter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pointcloud_to_pointcloud2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

