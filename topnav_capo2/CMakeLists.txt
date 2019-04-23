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
#        message_runtime
#        gazebo_msgs
        topnav_msgs
        topnav_shared
)

include_directories(${catkin_INCLUDE_DIRS})

#add_executable(capo_laser_scan_transceiver
#        src/capo_line_follower/hough_lidar.h
#        src/capo_line_follower/hough_lidar.cpp
#        src/capo_line_follower/LaserScanTransceiver.h
#        src/capo_line_follower/LaserScanTransceiver.cpp
#        src/capo_line_follower/laser_scan_transceiver.cpp
#)
#target_link_libraries(capo_laser_scan_transceiver ${catkin_LIBRARIES})
