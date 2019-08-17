#include <cv.hpp>
#include "ArUcoLocator.h"
#include <iostream>
#include <utility>
#include <ros/init.h>
#include <topnav_msgs/MarkersMsg.h>
#include <constants/topic_names.h>

ArUcoLocator::ArUcoLocator() {
    init();
}


Vec3d ArUcoLocator::retrieveMarkerReferenceFrameLocation(Marker marker) {
    return ArUcoLocator::retrieveMarkerReferenceFrameLocation(marker, nullptr);
}

Vec3d ArUcoLocator::retrieveMarkerReferenceFrameLocation(Marker marker, double *distance) {
    Vec3d rotation = marker.getRotation();
    Vec3d translation = marker.getTranslation();

    Mat cameraPosition = calculatePosition(rotation, translation);

    return cameraPosition;
}

Mat
ArUcoLocator::calculatePosition(const Vec3d &rotation, const Vec3d &translation) {
    Mat R;
    Rodrigues(rotation, R);
    Mat cameraPosition = -R.t() * Mat(translation);

    return cameraPosition;
}

void ArUcoLocator::printLocation(Vec3d location) {
    ROS_INFO("(%7.2f, %7.2f, %7.2f)", location.val[0], location.val[1], location.val[2]);
}

void ArUcoLocator::init() {
    detecor_subscriber = handle.subscribe(TOPIC_NAME_ARUCO_DETECTION, 1000, &ArUcoLocator::marker_scan_callback, this);
}

void ArUcoLocator::marker_scan_callback(const topnav_msgs::MarkersMsg::ConstPtr &msg) {
    vector<Marker> markers;
    vector<int> ids;
    vector<vector<Point2f>> allCorners;
    vector<Vec3d> rotations;
    vector<Vec3d> translations;

    vector<Point2f> corners;

    for (auto marker_msg : msg->markers) {
        ids.push_back(marker_msg.id);

        for (int j = 0; j < 4; j++) {
            Point2f corner(static_cast<float>(marker_msg.x_corners[j]), static_cast<float>(marker_msg.y_corners[j]));
            corners.push_back(corner);
        }
        rotations.emplace_back(marker_msg.rotation[0], marker_msg.rotation[1], marker_msg.rotation[2]);
        translations.emplace_back(marker_msg.translation[0], marker_msg.translation[1], marker_msg.translation[2]);

        corners.clear();
        allCorners.push_back(corners);
    }

    markers = Marker::convertToMarkers(ids, allCorners, rotations, translations);

    for (const auto &marker : markers) {
        Vec3d location = retrieveMarkerReferenceFrameLocation(marker);
        printLocation(location);
    }
}

double ArUcoLocator::calculateDistance(Vec3d position) {
    return norm(position);
}
