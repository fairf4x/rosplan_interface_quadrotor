cmake_minimum_required(VERSION 2.8.3)
project(rosplan_interface_quadrotor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    std_srvs
    genmsg
    mongodb_store
    actionlib
    geometry_msgs
)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rosplan_interface_quadrotor src/rosplan_interface_quadrotor.cpp)
target_link_libraries(rosplan_interface_quadrotor ${catkin_LIBRARIES})
