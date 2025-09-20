cmake_minimum_required(VERSION 3.0.2)
project(localizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  beacon_detector
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs beacon_detector
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(localizer_node src/localizer_node.cpp src/main.cpp)
target_link_libraries(localizer_node ${catkin_LIBRARIES})
add_dependencies(localizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
