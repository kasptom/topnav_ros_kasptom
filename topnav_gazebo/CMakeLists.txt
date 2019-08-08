SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
cmake_minimum_required(VERSION 2.8.3)
project(topnav_gazebo)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        gazebo_msgs
        message_generation
        topnav_msgs
        topnav_shared
        )

add_service_files(
        DIRECTORY srv
        FILES ResetWorld.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)


catkin_package(
        CATKIN_DEPENDS
        message_runtime
        gazebo_msgs
        topnav_msgs
        topnav_shared
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(world_reset_publisher src/world_reset/world_reset_publisher.cpp src/world_reset/topnav_world_reset.h)
target_link_libraries(world_reset_publisher ${catkin_LIBRARIES})

add_executable(world_reset_subscriber src/world_reset/world_reset_subscriber.cpp src/world_reset/topnav_world_reset.h)
target_link_libraries(world_reset_subscriber ${catkin_LIBRARIES})

add_executable(laser_scan_converter
        src/laser_scan_converter/hough_lidar.h
        src/laser_scan_converter/hough_lidar.cpp
        src/laser_scan_converter/LaserScanConverter.h
        src/laser_scan_converter/LaserScanConverter.cpp
        src/laser_scan_converter/laser_scan_converter.cpp
        )
target_link_libraries(laser_scan_converter ${catkin_LIBRARIES})
