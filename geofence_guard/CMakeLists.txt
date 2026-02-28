cmake_minimum_required(VERSION 3.0.2)
project(geofence_guard)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  mavros_msgs
  std_msgs
)

## System dependencies
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    geometry_msgs 
    nav_msgs 
    mavros_msgs 
    std_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(geofence_guard_node 
  src/main.cpp
  src/geofence_guard.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(geofence_guard_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(geofence_guard_node
  ${catkin_LIBRARIES}
)
