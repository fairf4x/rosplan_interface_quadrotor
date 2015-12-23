cmake_minimum_required(VERSION 2.8.3)
project(test_quadrotor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg hector_path_follower)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(test_quadrotor src/test_quadrotor.cpp)
target_link_libraries(test_quadrotor ${catkin_LIBRARIES})
