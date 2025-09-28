cmake_minimum_required(VERSION 3.0.2)
project(viz_mux)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  localizer            # onde estão as msgs localizer/BeaconMatch
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(viz_mux_node src/viz_mux_node.cpp)
add_dependencies(viz_mux_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(viz_mux_node ${catkin_LIBRARIES})

install(TARGETS viz_mux_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
