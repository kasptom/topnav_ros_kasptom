#ifndef TOPNAV_GAZEBO_ARUCODETECTOR_H
#define TOPNAV_GAZEBO_ARUCODETECTOR_H

#include <sensor_msgs/Image.h>
#include <ros/node_handle.h>
#include <opencv2/aruco.hpp>

static const float MARKER_LENGTH_METERS = 0.17;
static const std::string ARUCO_OPENCV_WINDOW_NAME = "Aruco detection preview";

class ArUcoDetector {
public:
    ArUcoDetector(std::string cameraConfigFileName);

    void camera_image_callback(const sensor_msgs::Image::ConstPtr &msg);

    ~ArUcoDetector();

    bool readCameraParameters(std::string filename);

private:
    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;

    ros::NodeHandle handle;
    ros::Subscriber camera_subscriber;

    bool init(std::string fileName);
};


#endif //TOPNAV_GAZEBO_ARUCODETECTOR_H
