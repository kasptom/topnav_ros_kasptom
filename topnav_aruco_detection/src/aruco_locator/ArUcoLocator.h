#ifndef ARUCO_LOCATIONRETRIEVER_H
#define ARUCO_LOCATIONRETRIEVER_H

#include "../aruco_model/Marker.h"
#include <opencv2/highgui.hpp>
#include <fstream>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <topnav_msgs/Markers.h>

using namespace cv;
static const float MARKER_LENGTH_METERS = 0.17;

class ArUcoLocator {
public:
    ArUcoLocator();

    ros::NodeHandle handle;
    ros::Subscriber detecor_subscriber;

    Vec3d retrieveMarkerReferenceFrameLocation(Marker marker);
    Vec3d retrieveMarkerReferenceFrameLocation(Marker marker, double *distance);

    /**
     * Prints camera coordinates with two decimal points accuracy
     * @param location
     */
    void printLocation(Vec3d location);

private:
    void init();
    Mat calculatePosition(const Vec3d &rotation, const Vec3d &translation, double* distance) const;

    void marker_scan_callback(const boost::shared_ptr<const topnav_msgs::Markers_ <allocator<void>>> &msg);
};


#endif //ARUCO_LOCATIONRETRIEVER_H
