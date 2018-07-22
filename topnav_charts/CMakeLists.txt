SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
cmake_minimum_required(VERSION 2.8.3)
project(topnav_charts)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# SFML
set(CMAKE_MODULE_PATH "/usr/share/SFML/cmake/Modules;${CMAKE_MODULE_PATH}")

find_package(catkin REQUIRED
        roscpp
        rospy
        topnav_msgs
        )

find_package(SFML 2 REQUIRED graphics window system)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        topnav_msgs
)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(topnav_lines_preview_node
        src/topnav_lines_preview_node.cpp
        src/topnav_lines_preview_node.h
        src/HoughLinesPreview.cpp
        src/HoughLinesPreview.h
        )

if(SFML_FOUND)
    include_directories(${SFML_INCLUDE_DIR})
    target_link_libraries(topnav_lines_preview_node ${SFML_LIBRARIES} ${SFML_DEPENDENCIES} ${catkin_LIBRARIES})
endif()
