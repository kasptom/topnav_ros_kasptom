#include <sensor_msgs/Image.h>
#include "ArUcoDetector.h"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"

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

    camera_subscriber = handle.subscribe("capo/camera1/image_raw", 1000, &ArUcoDetector::camera_image_callback, this);
    namedWindow(ARUCO_OPENCV_WINDOW_NAME);
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
    return ArUcoDetector::readCameraParameters(filename);
}

bool ArUcoDetector::readCameraParameters(std::string filename) {
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distortionCoefficients;
    return true;
}

int main(int argc, char **argv) {
    string cameraConfigFilePath = argc > 1 ? argv[1] : "c930.yaml";

    ROS_INFO("Config file: %s", cameraConfigFilePath.c_str());

    ros::init(argc, argv, "aruco_detector");
    ArUcoDetector detector(cameraConfigFilePath);
    ros::spin();
    return 0;
}