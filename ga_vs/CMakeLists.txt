cmake_minimum_required(VERSION 2.8.3)
project(ga_vs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED) 

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


## Declare a C++ executable
add_executable(vs_control_node src/vs_control.cpp)
# add_executable(${PROJECT_NAME}_node src/IBVS_controller_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(vs_control_node ${catkin_LIBRARIES})
