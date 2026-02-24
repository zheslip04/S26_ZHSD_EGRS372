cmake_minimum_required(VERSION 3.0.2)
project(egrs372_lab5)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES egrs372_lab5
#  CATKIN_DEPENDS message_generation message_runtime roscpp std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Existing executable
add_executable(slam_nav src/slam_nav.cpp)
target_link_libraries(slam_nav
  ${catkin_LIBRARIES}
)

add_dependencies(slam_nav ${catkin_EXPORTED_TARGETS})

