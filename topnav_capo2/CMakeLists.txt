SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
cmake_minimum_required(VERSION 2.8.3)
project(topnav_capo2)

find_package(catkin REQUIRED COMPONENTS
        rospy
        std_msgs
        message_generation
        topnav_msgs
        topnav_shared
)

catkin_package(
        CATKIN_DEPENDS
        topnav_msgs
        topnav_shared
)

include_directories(${catkin_INCLUDE_DIRS})
