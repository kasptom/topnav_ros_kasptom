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
        topnav_shared
        gazebo_msgs
        )

find_package(SFML 2 REQUIRED graphics window system)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        topnav_msgs
        topnav_shared
        gazebo_msgs
)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(hough_space_preview
        src/hough_space_preview_node.cpp
        src/hough_space_preview_node.h
        src/HoughSpacePreview.cpp
        src/HoughSpacePreview.h)

add_executable(lines_preview
        src/lines_preview_node.cpp
        src/lines_preview_node.h
        src/LinesPreview.cpp
        src/LinesPreview.h)

if(SFML_FOUND)
    include_directories(${SFML_INCLUDE_DIR})
    target_link_libraries(hough_space_preview ${SFML_LIBRARIES} ${SFML_DEPENDENCIES} ${catkin_LIBRARIES})
    target_link_libraries(lines_preview ${SFML_LIBRARIES} ${SFML_DEPENDENCIES} ${catkin_LIBRARIES})
endif()
