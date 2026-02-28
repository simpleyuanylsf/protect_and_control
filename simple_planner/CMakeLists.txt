cmake_minimum_required(VERSION 3.0.2)
project(simple_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} # 需要确保系统安装了 Eigen3
)

add_executable(uavkey_control_node src/main.cpp src/uavkey_control.cpp)
target_link_libraries(uavkey_control_node ${catkin_LIBRARIES})

# 添加键盘控制节点
add_executable(keyboard_control_node src/keyboard_control.cpp)
target_link_libraries(keyboard_control_node ${catkin_LIBRARIES})