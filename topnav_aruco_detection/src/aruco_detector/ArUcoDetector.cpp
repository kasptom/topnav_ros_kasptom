#include <sensor_msgs/Image.h>
#include "ArUcoDetector.h"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "../aruco_model/Marker.h"
#include <FileUtils.h>

using namespace cv;
using namespace std;

ArUcoDetector::ArUcoDetector(string cameraConfigFileName) {
    bool initOk = init(cameraConfigFileName);
    if (!initOk) {
        ROS_ERROR("Invalid camera file: %s\n specify the full path of the camera config file (c930.yaml)",
                  cameraConfigFileName.c_str());
        return;
    } else {
        ROS_INFO("Successfully set camera config: %s", cameraConfigFileName.c_str());
    }

    camera_subscriber = nodeHandle.subscribe("capo/camera1/image_raw", 1000, &ArUcoDetector::camera_image_callback, this);
    detection_publisher = nodeHandle.advertise<topnav_msgs::MarkersMsg>(TOPIC_NAME_ARUCO_DETECTION, 1000);
}

/**
 * http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 * @param msg
 */
void ArUcoDetector::camera_image_callback(const sensor_msgs::Image::ConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    int dictionaryId = aruco::DICT_6X6_250;

    Ptr<aruco::DetectorParameters> detectorParameters = aruco::DetectorParameters::create();
    detectorParameters->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;

    vector<int> ids;
    vector<vector<Point2f> > corners, rejected;
    vector<Vec3d> rVectors, tVectors;
    Mat image(cv_ptr->image);

//    ROS_INFO("height = %d, width = %d", msg->height, msg->width);

    try {
        Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(
                aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParameters, rejected);

        long numberOfMarkers = ids.size();
        if (numberOfMarkers > 0) {
            aruco::estimatePoseSingleMarkers(
                    corners, MARKER_LENGTH_METERS, cameraMatrix, distortionCoefficients, rVectors, tVectors);
        }

        if (numberOfMarkers > 0) {
            aruco::drawDetectedMarkers(image, corners, ids);

            for (int i = 0; i < ids.size(); i++) {
                aruco::drawAxis(
                        image, cameraMatrix, distortionCoefficients, rVectors[i], tVectors[i],
                        MARKER_LENGTH_METERS * 0.5f);
            }

            topnav_msgs::MarkersMsg markers_msg = ArUcoDetector::create_marker_detection_message(ids, corners, rVectors, tVectors);
            detection_publisher.publish(markers_msg);
        }

    } catch (cv::Exception &exception) {
        cerr << exception.msg << endl;
    }

    // Update GUI Window
    imshow(ARUCO_OPENCV_WINDOW_NAME, cv_ptr->image);
    waitKey(3);
}

ArUcoDetector::~ArUcoDetector() {
    cv::destroyWindow(ARUCO_OPENCV_WINDOW_NAME);
}

bool ArUcoDetector::init(std::string filename) {
    return ArUcoDetector::readCameraParameters(std::move(filename));
}

bool ArUcoDetector::readCameraParameters(std::string filename) {
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distortionCoefficients;
    return true;
}

topnav_msgs::MarkersMsg
ArUcoDetector::create_marker_detection_message(std::vector<int> ar_uco_ids, std::vector<std::vector<cv::Point2f>> corners,
                                               std::vector<cv::Vec3d> rvectors, std::vector<cv::Vec3d> tvectors) {
    topnav_msgs::MarkersMsg markers_msg;

    for (int i = 0; i < ar_uco_ids.size(); i++){
        topnav_msgs::MarkerMsg marker;
        marker.id = ar_uco_ids[i];

        vector<cv::Point2f> marker_corners = corners[i];
        for (int j = 0; j < 3; j++)
        {
            marker.rotation.push_back(rvectors[i][j]);
            marker.translation.push_back(tvectors[i][j]);
        }

        for (int j = 0; j < 4; j++)
        {
            marker.x_corners.push_back(marker_corners[j].x);
            marker.y_corners.push_back(marker_corners[j].y);
        }

        markers_msg.markers.push_back(marker);
    }

    return markers_msg;
}

int main(int argc, char **argv) {
    string cameraConfigFilePath;
    if (argc > 1) {
        cameraConfigFilePath = argv[1];
    } else {
        cameraConfigFilePath = FileUtils::get_file_path_under_exe_dir("c930.yaml");
    }

    ROS_INFO("Config file: %s", cameraConfigFilePath.c_str());

    ros::init(argc, argv, "aruco_detector");
    ArUcoDetector detector(cameraConfigFilePath);
    ros::spin();
    return 0;
}