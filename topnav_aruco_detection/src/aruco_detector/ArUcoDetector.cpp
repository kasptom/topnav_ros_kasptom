#include <sensor_msgs/Image.h>
#include "ArUcoDetector.h"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "../aruco_model/Marker.h"
#include "../aruco_locator/ArUcoLocator.h"
#include <FileUtils.h>

using namespace cv;
using namespace std;

ArUcoDetector::ArUcoDetector(std::string cameraConfigFileName, std::string arUcoSizesFilePath, bool visualize) {
    bool initOk = init(cameraConfigFileName);
    magicArUcoCoefficient = MAGIC_COEFFICIENT;

    if (!initOk) {
        ROS_ERROR("Invalid camera file: %s\n specify the full path of the camera config file (c930.yaml)",
                  cameraConfigFileName.c_str());
        return;
    } else {
        ROS_INFO("Successfully set camera config: %s", cameraConfigFileName.c_str());
    }

    initOk = initArUcos(arUcoSizesFilePath);
    if (!initOk) {
        ROS_ERROR("Invalid ArUco sizes file: %s\n specify the full path of the camera config file (*.txt)",
                  arUcoSizesFilePath.c_str());
        return;
    } else {
        ROS_INFO("Successfully set ArUco sizes file: %s", arUcoSizesFilePath.c_str());
    }

    this->visualize = visualize;
    camera_subscriber = nodeHandle.subscribe(TOPIC_NAME_CAPO_CAMERA_RAW_IMAGE, 1, &ArUcoDetector::camera_image_callback, this);
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
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParameters, rejected, cameraMatrix, distortionCoefficients);

        long numberOfMarkers = ids.size();
        if (numberOfMarkers > 0) {
            aruco::estimatePoseSingleMarkers(
                    corners, MARKER_LENGTH_METERS, cameraMatrix, distortionCoefficients, rVectors, tVectors);
        }

        if (numberOfMarkers > 0 && visualize) {
            aruco::drawDetectedMarkers(image, corners, ids);

            for (int i = 0; i < ids.size(); i++) {
                aruco::drawAxis(
                        image, cameraMatrix, distortionCoefficients, rVectors[i], tVectors[i],
                        MARKER_LENGTH_METERS);
            }

        }

        topnav_msgs::MarkersMsg markers_msg = ArUcoDetector::create_marker_detection_message(ids, corners, rVectors, tVectors);
        detection_publisher.publish(markers_msg);

    } catch (cv::Exception &exception) {
        cerr << exception.msg << endl;
        ROS_ERROR("Error [%d] during aruco detection at line %d, %s", exception.code, exception.line, exception.msg.c_str());
    }

    if (visualize) {
        // Update GUI Window
        imshow(ARUCO_OPENCV_WINDOW_NAME, cv_ptr->image);
        waitKey(3);
    }
}

ArUcoDetector::~ArUcoDetector() {
    if (visualize) {
        cv::destroyWindow(ARUCO_OPENCV_WINDOW_NAME);
    }
}

bool ArUcoDetector::init(std::string filename) {
    return ArUcoDetector::readCameraParameters(std::move(filename));
}


bool ArUcoDetector::initArUcos(std::string arUcoSizesFileName) {
    loadArUcoSizes(std::move(arUcoSizesFileName));
    return true;
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

        double distance;
        Vec3d camera_position = ArUcoLocator::calculatePosition(rvectors[i], tvectors[i], &distance);
        resizeMarker(camera_position, arUcoSizesMap[marker.id]);
//        print_position(const_cast<char *>("1st position"), camera_position, distance);

        vector<cv::Point2f> marker_corners = corners[i];
        for (int j = 0; j < 3; j++)
        {
            marker.rotation.push_back(rvectors[i][j]);
            marker.translation.push_back(tvectors[i][j]);
            marker.camera_position.push_back(camera_position[j]);
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

void ArUcoDetector::loadArUcoSizes(std::string file_name) {
    string line;
    ifstream arUcoFile (file_name);

    if (arUcoFile.is_open())
    {
        char * pEnd;
        while ( getline (arUcoFile,line) )
        {
            auto arUcoId = static_cast<int>(std::strtol(line.c_str(), &pEnd, 10));
            double arUcoSize = std::strtod(pEnd, nullptr);

            if (arUcoId == MAGIC_COEFFICIENT_ID) {
                magicArUcoCoefficient = arUcoSize;
            } else {
                arUcoSizesMap[arUcoId] = arUcoSize;
            }
        }
        arUcoFile.close();
    }

    else cout << "Unable to open file";
}

void ArUcoDetector::resizeMarker(cv::Vec3d &cameraPosition, double &realMarkerSize) {
    for (int i = 0; i < 3; i++) {
        cameraPosition[i] *= (realMarkerSize / MARKER_LENGTH_METERS) * magicArUcoCoefficient;
    }
}

void ArUcoDetector::print_position(char *label, cv::Vec3d &cameraPosition, double distance) {
    ROS_INFO("%s (%.2f, %.2f, %.2f), d=%.2f", label, cameraPosition[0], cameraPosition[1], cameraPosition[2], distance);
}

int main(int argc, char **argv) {
    string cameraConfigFilePath;
    string arUcoSizesFilePath;
    bool visualize = true;

    /* argv[0], and argv[argc - 1] and argv[argc - 2] are reserved for ROS
     (node file path, node name and log file path respectively)
    for (int i = 0; i < argc; i++) {
        ROS_INFO("argv[%d]=%s", i, argv[i]);
    }
    */

    if (argc > 3) {
        cameraConfigFilePath = argv[1];
    }

    if (argc > 4) {
        arUcoSizesFilePath = argv[2];
    }

    if (argc > 5) {
        visualize = strcmp("true", argv[3]) == 0;
    }


    if (cameraConfigFilePath.find(".yaml") == string::npos) {
        cameraConfigFilePath = FileUtils::get_file_path_under_exe_dir("c930.yaml");
    }

    if (arUcoSizesFilePath.find(".txt") == string::npos) {
        arUcoSizesFilePath = FileUtils::get_file_path_under_exe_dir("aruco_sizes.txt");
    }

    ROS_INFO("config file: %s", cameraConfigFilePath.c_str());
    ROS_INFO("ArUco sizes file: %s", arUcoSizesFilePath.c_str());
    ROS_INFO("visualize: %s", visualize ? "true" : "false");

    ros::init(argc, argv, "aruco_detector");

    ArUcoDetector detector(cameraConfigFilePath, arUcoSizesFilePath, visualize);
    ros::spin();
    return 0;
//    // ---
//
//    int loopFrequencyHz = 30;
//
//    ROS_INFO("publishing with %d [Hz] frequency", loopFrequencyHz);
//
//    ros::Rate rate(loopFrequencyHz); // Hz
//
//    while (!ros::isShuttingDown()) {
//        ros::spinOnce();
//        rate.sleep();
//    }
//    // ---
}