#include <cv.hpp>
#include "ArUcoLocator.h"
#include <iostream>
#include <ros/init.h>

ArUcoLocator::ArUcoLocator() {
    init();
}


Vec3d ArUcoLocator::retrieveMarkerReferenceFrameLocation(Marker marker) {
    return ArUcoLocator::retrieveMarkerReferenceFrameLocation(marker, nullptr);
}

Vec3d ArUcoLocator::retrieveMarkerReferenceFrameLocation(Marker marker, double *distance) {
    Vec3d rotation = marker.getRotation();
    Vec3d translation = marker.getTranslation();

    Mat cameraPosition = calculatePosition(rotation, translation, distance);
    return cameraPosition;
}

Mat
ArUcoLocator::calculatePosition(const Vec3d &rotation, const Vec3d &translation, double *distance) const {
    Mat R;
    Rodrigues(rotation, R);
    Mat cameraPosition = -R.t() * Mat(translation);

    if (distance != nullptr) {
        *distance = norm(cameraPosition);
    }

    return cameraPosition;
}

void ArUcoLocator::printLocation(Vec3d location) {
    printf("(%7.2f, %7.2f, %7.2f)", location.val[0], location.val[1], location.val[2]);
}

void ArUcoLocator::init() {
    // TODO Markers message
//    detecor_subscriber = handle.subscribe("capo/camera1/image_raw", 1000, &ArUcoLocator::marker_scan_callback, this);
}

void ArUcoLocator::marker_scan_callback() {
    // TODO Markers message
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "aruco_locator");
    ArUcoLocator locator;
    ros::spin();
    return 0;
}