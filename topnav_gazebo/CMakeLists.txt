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
#        topnav_utils FIXME
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
#        topnav_utils FIXME
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(world_reset_publisher src/world_reset/world_reset_publisher.cpp src/world_reset/topnav_world_reset.h)
target_link_libraries(world_reset_publisher ${catkin_LIBRARIES})

add_executable(world_reset_subscriber src/world_reset/world_reset_subscriber.cpp src/world_reset/topnav_world_reset.h)
target_link_libraries(world_reset_subscriber ${catkin_LIBRARIES})

add_executable(capo_laser_scan_transceiver
        src/capo_line_follower/hough_lidar.h
        src/capo_line_follower/hough_lidar.cpp
        src/capo_line_follower/LaserScanTransceiver.h
        src/capo_line_follower/LaserScanTransceiver.cpp
        src/capo_line_follower/LaserParameters.cpp
        src/capo_line_follower/LaserParameters.h
        src/capo_line_follower/laser_scan_transceiver.cpp
        )
target_link_libraries(capo_laser_scan_transceiver ${catkin_LIBRARIES})