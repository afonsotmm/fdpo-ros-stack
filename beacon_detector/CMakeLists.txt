cmake_minimum_required(VERSION 3.0.2)
project(beacon_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  laser_geometry
  tf2_ros
  tf2_sensor_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(beacon_detector_node
  src/main.cpp
  src/beacon_detector_node.cpp
)

target_link_libraries(beacon_detector_node
  ${catkin_LIBRARIES}
)
