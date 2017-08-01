cmake_minimum_required(VERSION 2.8.3)
project(ibvs_controller)

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

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/IBVS_controller.cpp
# )


## Declare a C++ executable
add_executable(ibvs_controller_node src/IBVS.cpp)
add_executable(image_position_control_node src/image_position_control.cpp)
# add_executable(${PROJECT_NAME}_node src/IBVS_controller_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ibvs_controller_node ${catkin_LIBRARIES})
target_link_libraries(image_position_control_node ${catkin_LIBRARIES})
