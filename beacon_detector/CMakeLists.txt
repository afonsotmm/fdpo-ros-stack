cmake_minimum_required(VERSION 3.0.2)
project(beacon_detector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  laser_geometry
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs       
  visualization_msgs
  geometry_msgs             
  xmlrpcpp    
  message_generation             
)

add_message_files(
  FILES
    Pose.msg
    Cluster.msg
    BeaconMatch.msg
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    laser_geometry
    tf2_ros
    tf2_sensor_msgs
    tf2_geometry_msgs      
    visualization_msgs
    geometry_msgs          
    xmlrpcpp       
    message_runtime         
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(beacon_detector_node
  src/main.cpp
  src/beacon_detector_node.cpp
  src/dbscan.cpp
  src/cluster.cpp
  src/point.cpp
)

add_dependencies(beacon_detector_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(beacon_detector_node
  ${catkin_LIBRARIES}
)

