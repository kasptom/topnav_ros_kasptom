#ifndef TOPNAV_GAZEBO_ARUCODETECTOR_H
#define TOPNAV_GAZEBO_ARUCODETECTOR_H

#include <sensor_msgs/Image.h>
#include <ros/node_handle.h>

static const float MARKER_LENGTH_METERS = 0.17;

class ArUcoDetector {
public:
    ArUcoDetector();

    void camera_image_callback(const sensor_msgs::Image::ConstPtr &msg);

private:
    ros::NodeHandle handle;
    ros::Subscriber camera_subscriber;
};


#endif //TOPNAV_GAZEBO_ARUCODETECTOR_H
