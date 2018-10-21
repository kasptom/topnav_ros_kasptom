#include <sensor_msgs/Image.h>
#include "ArUcoDetector.h"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <boost/thread/mutex.hpp>

using namespace cv;
using namespace std;

ArUcoDetector::ArUcoDetector() {
    camera_subscriber = handle.subscribe("capo/camera1/image_raw", 1000, &ArUcoDetector::camera_image_callback, this);
}

void ArUcoDetector::camera_image_callback(const sensor_msgs::Image::ConstPtr &msg) {
    int dictionaryId = aruco::DICT_6X6_250;
    bool showRejected = false;
    bool estimatePose = true;

    Ptr<aruco::DetectorParameters> detectorParameters = aruco::DetectorParameters::create();
    detectorParameters->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;

    vector<int> ids;
    vector< vector<Point2f> > corners, rejected;
    vector<Vec3d> rVectors, tVectors;
    Mat imageCopy;
    Mat image(msg->data);
    Mat cameraMatrix, distortionCoefficients;

    ROS_INFO("height = %d, width = %d", msg->height, msg->width);

    // TODO use opencv_bridge from ROS' vision_opencv
//    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
//
//    aruco::detectMarkers(image, dictionary, corners, ids, detectorParameters, rejected);
//
//    if (ids.size() > 0) {
//        long numberOfMarkers = ids.size();
//        aruco::estimatePoseSingleMarkers(corners, MARKER_LENGTH_METERS, cameraMatrix, distortionCoefficients,
//                                         rVectors, tVectors);
//    }
//
//    image.copyTo(imageCopy);
//
//    if (ids.size() > 0) {
//        aruco::drawDetectedMarkers(imageCopy, corners, ids);
//
//        if (estimatePose) {
//            for (int i = 0; i < ids.size(); i++) {
//                aruco::drawAxis(imageCopy, cameraMatrix, distortionCoefficients, rVectors[i], tVectors[i],
//                                MARKER_LENGTH_METERS * 0.5f);
//            }
//        }
//    }
//
//    if (showRejected && rejected.size() > 0) {
//        aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));
//    }
//
//    imshow("out", imageCopy);
}
