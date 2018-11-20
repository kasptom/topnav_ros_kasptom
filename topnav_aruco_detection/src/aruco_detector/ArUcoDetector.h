#ifndef TOPNAV_GAZEBO_ARUCODETECTOR_H
#define TOPNAV_GAZEBO_ARUCODETECTOR_H

#include <sensor_msgs/Image.h>
#include <ros/node_handle.h>
#include <opencv2/aruco.hpp>
#include <topnav_msgs/Markers.h>
#include <constants/topic_names.h>

static const float MARKER_LENGTH_METERS = 0.17;

static const std::string ARUCO_OPENCV_WINDOW_NAME = "Aruco detection preview"; // NOLINT

class ArUcoDetector {
public:
    ArUcoDetector(std::string cameraConfigFileName);

    void camera_image_callback(const sensor_msgs::Image::ConstPtr &msg);

    ~ArUcoDetector();

    bool readCameraParameters(std::string filename);

    topnav_msgs::Markers create_marker_detection_message(std::vector<int> ar_uco_ids, std::vector<std::vector<cv::Point2f>> corners,
                                                             std::vector<cv::Vec3d> rvectors, std::vector<cv::Vec3d> tvectors);

private:
    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;

    ros::NodeHandle nodeHandle;
    ros::Subscriber camera_subscriber;
    ros::Publisher detection_publisher;

    bool init(std::string fileName);
};


#endif //TOPNAV_GAZEBO_ARUCODETECTOR_H
