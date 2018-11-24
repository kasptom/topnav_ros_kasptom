#ifndef TOPNAV_GAZEBO_LASERSCANTRANSCEIVER_H
#define TOPNAV_GAZEBO_LASERSCANTRANSCEIVER_H


#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <topnav_msgs/HoughAcc.h>
#include <ros/node_handle.h>
#include <models/LaserParameters.h>

static const std::string TOPIC_NAME_LASER_TRANSCEIVER = "/capo/laser/hough"; // NOLINT

class LaserScanTransceiver {
public:
    LaserScanTransceiver();

    void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

private:
    ros::NodeHandle handle;
    ros::Publisher hough_space_publisher;
    ros::Subscriber laser_scan_subscriber;
    LaserParameters parameters = LaserParameters(0, 0, 0, 0, 0);

    topnav_msgs::HoughAcc create_hough_message(std::vector<std::vector<int>> rhoThetaMatrix);
};


#endif //TOPNAV_GAZEBO_LASERSCANTRANSCEIVER_H
