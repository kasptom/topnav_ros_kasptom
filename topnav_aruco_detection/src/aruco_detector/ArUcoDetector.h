#ifndef TOPNAV_GAZEBO_ARUCODETECTOR_H
#define TOPNAV_GAZEBO_ARUCODETECTOR_H

#include <sensor_msgs/Image.h>
#include <ros/node_handle.h>
#include <opencv2/aruco.hpp>
#include <topnav_msgs/MarkersMsg.h>
#include <constants/topic_names.h>

static const float MARKER_LENGTH_METERS = 0.15;

//static const float MAGIC_COEFFICIENT = 0.46; // 320x240, 2017
//static const float MAGIC_COEFFICIENT = 0.935; // 640x480, 2017
static const float MAGIC_COEFFICIENT = 1.0; // 640x480, 2019-08-17

static const float MAGIC_COEFFICIENT_ID = -56;

static const std::string ARUCO_OPENCV_WINDOW_NAME = "Aruco detection preview"; // NOLINT

class ArUcoDetector {
public:
    ArUcoDetector(std::string cameraConfigFileName, std::string arUcoSizesFilePath, bool visualize, bool printDistance);

    void camera_image_callback(const sensor_msgs::Image::ConstPtr &msg);

    ~ArUcoDetector();

    bool readCameraParameters(std::string filename);

    topnav_msgs::MarkersMsg create_marker_detection_message(std::vector<int> ar_uco_ids, std::vector<std::vector<cv::Point2f>> corners,
                                                             std::vector<cv::Vec3d> rvectors, std::vector<cv::Vec3d> tvectors);

private:
    cv::Mat cameraMatrix;
    cv::Mat distortionCoefficients;

    std::map<int, double> arUcoSizesMap;
    double magicArUcoCoefficient;

    ros::NodeHandle nodeHandle;
    ros::Subscriber camera_subscriber;
    ros::Publisher detection_publisher;

    bool visualize;
    bool printDistance;

    bool init(std::string fileName);

    bool initArUcos(std::string arUcoSizesFileName);

    void loadArUcoSizes(std::string file_name);

    void resizeMarker(cv::Vec3d &cameraPosition, double &realMarkerSize);

    void printPosition(char *label, cv::Vec3d &cameraPosition, double d);
};


#endif //TOPNAV_GAZEBO_ARUCODETECTOR_H
