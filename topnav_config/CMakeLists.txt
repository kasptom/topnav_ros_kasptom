cmake_minimum_required(VERSION 2.8.3)
project(topnav_config)

find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        roscpp
        rospy
        topnav_msgs
        topnav_shared
        )

generate_dynamic_reconfigure_options(
        cfg/topnav.cfg
)

catkin_package(CATKIN_DEPENDS
        dynamic_reconfigure
        roscpp rospy topnav_msgs
        topnav_shared
)


include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/topnav_config_node.cpp src/ConfigNode.cpp src/ConfigNode.h)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

add_dependencies(topnav_config_node ${PROJECT_NAME}_gencfg)
