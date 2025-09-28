cmake_minimum_required(VERSION 3.0.2)
project(odom_path_tools)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  visualization_msgs
  geometry_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(odoms_to_paths_node src/odoms_to_paths_node.cpp)
target_link_libraries(odoms_to_paths_node ${catkin_LIBRARIES})
add_dependencies(odoms_to_paths_node ${catkin_EXPORTED_TARGETS})
