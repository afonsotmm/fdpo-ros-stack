cmake_minimum_required(VERSION 3.0.2)
project(navigation_controller)

# C++11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation       
)

## ---- serviços/messages ----
add_service_files(
  FILES
  NavigationControl.srv    
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## ---- pacote ----
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_runtime       
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## ---- executável ----
add_executable(navigation_controller_node
  src/main.cpp
  src/navigation_controller_node.cpp
  src/fsm.cpp
)

# Garante que o node só compila após gerar o header do serviço
add_dependencies(navigation_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(navigation_controller_node
  ${catkin_LIBRARIES}
)
