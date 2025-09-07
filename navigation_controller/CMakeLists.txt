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
)

# Exporta include/ para quem (no futuro) usar as tuas headers
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES (nenhuma lib própria por agora)
  # CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Executável com TODOS os fontes
add_executable(navigation_controller_node
  src/main.cpp
  src/navigation_controller_node.cpp
  src/fsm.cpp
)

add_dependencies(navigation_controller_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(navigation_controller_node
  ${catkin_LIBRARIES}
)

# (Opcional) instalação
# install(TARGETS navigation_controller_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# install(DIRECTORY launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
