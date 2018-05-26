cmake_minimum_required(VERSION 2.8.3)
project(topnav_gazebo)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs)

catkin_package(
        CATKIN_DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(world_reset_publisher world_reset/src/msg/world_reset_publisher.cpp world_reset/src/msg/topnav_world_reset.h)
target_link_libraries(world_reset_publisher ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(world_reset_subscriber world_reset/src/msg/world_reset_subscriber.cpp world_reset/src/msg/topnav_world_reset.h)
target_link_libraries(world_reset_subscriber ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)
