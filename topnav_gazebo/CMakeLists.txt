cmake_minimum_required(VERSION 2.8.3)
project(topnav_gazebo)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        gazebo_msgs
        message_generation
)

#add_message_files(
#        DIRECTORY msg
#        FILES
#        Num.msg
#)

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
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(world_reset_publisher src/world_reset/world_reset_publisher.cpp src/world_reset/topnav_world_reset.h)
target_link_libraries(world_reset_publisher ${catkin_LIBRARIES})

add_executable(world_reset_subscriber src/world_reset/world_reset_subscriber.cpp src/world_reset/topnav_world_reset.h)
target_link_libraries(world_reset_subscriber ${catkin_LIBRARIES})

add_executable(capo_line_follower src/capo_line_follower/capo_line_follower.cpp)
target_link_libraries(capo_line_follower ${catkin_LIBRARIES})