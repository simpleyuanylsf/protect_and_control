cmake_minimum_required(VERSION 3.0.2)
project(low_battery)

# 1. 设置 C++ 标准 (推荐 C++14 或 11)
add_compile_options(-std=c++14)

# 2. 查找 ROS 依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
  sensor_msgs
  std_msgs
)

# 3. 查找 Eigen3 库 (用于矩阵运算和距离计算)
find_package(Eigen3 REQUIRED)

# 4. 声明 Catkin 包
catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES drone_safety
  CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs sensor_msgs std_msgs
#  DEPENDS system_lib
)

# 5. 包含头文件目录 (包括 ROS 和 Eigen)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 6. 编译可执行文件
add_executable(low_battery src/low_battery.cpp)

# 7. 链接库
target_link_libraries(low_battery
  ${catkin_LIBRARIES}
  # Eigen 是纯头文件库，通常不需要显式链接库文件，但如果有依赖需加上
)