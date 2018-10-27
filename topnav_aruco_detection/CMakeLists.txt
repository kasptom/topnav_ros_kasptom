cmake_minimum_required(VERSION 2.8.3)
project(topnav_aruco_detection)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES topnav_aruco_detection
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV REQUIRED)

file(COPY launch/ DESTINATION ${CATKIN_DEVEL_PREFIX}/lib/topnav_aruco_detection)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(aruco_detector
        src/aruco_detector/ArUcoDetector.h
        src/aruco_detector/ArUcoDetector.cpp)
target_link_libraries(aruco_detector ${catkin_LIBRARIES} ${OpenCV_LIBS})


