#ifndef ARUCO_LOCATIONRETRIEVER_H
#define ARUCO_LOCATIONRETRIEVER_H

#include "../aruco_model/Marker.h"
#include <opencv2/highgui.hpp>
#include <fstream>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <topnav_msgs/MarkersMsg.h>

using namespace cv;

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

    static Mat calculatePosition(const Vec3d &rotation, const Vec3d &translation);

    static double calculateDistance(Vec3d position);

private:
    void init();

    void marker_scan_callback(const boost::shared_ptr<const topnav_msgs::MarkersMsg_ <allocator<void>>> &msg);
};


#endif //ARUCO_LOCATIONRETRIEVER_H
