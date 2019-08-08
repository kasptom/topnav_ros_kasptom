#ifndef TOPNAV_QN2QL_TOPIC_NAMES_H
#define TOPNAV_QN2QL_TOPIC_NAMES_H

#include <iostream>

static const std::string TOPIC_NAME_TOPNAV_CONFIG = "/topnav/config"; // NOLINT
static const std::string TOPIC_NAME_CAPO_CAMERA_RAW_IMAGE  ="/capo/camera1/image_raw"; // NOLINT

static const std::string TOPIC_NAME_CAPO_LASER_SCAN = "/capo/laser/scan"; // NOLINT

static const std::string TOPIC_NAME_CAPO_LASER_HOUGH = "/converter/laser/hough"; // NOLINT
static const std::string TOPIC_NAME_LASER_ANGLE_RANGE = "/converter/laser/angle_range"; // NOLINT
static const std::string TOPIC_NAME_ARUCO_DETECTION = "/converter/camera1/aruco"; // NOLINT

#endif //TOPNAV_QN2QL_TOPIC_NAMES_H
