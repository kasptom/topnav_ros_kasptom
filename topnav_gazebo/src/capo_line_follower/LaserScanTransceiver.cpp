#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include "LaserScanTransceiver.h"
#include "hough_lidar.h"

LaserScanTransceiver::LaserScanTransceiver() {
    sensor_msgs::LaserScan::ConstPtr ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/capo/laser/scan");
    readLaserParameters(ptr);

    ros::NodeHandle handle;
    laserScanSubscriber = handle.subscribe("/capo/laser/scan", 1000, &LaserScanTransceiver::laserScanCallback, this);
}

void LaserScanTransceiver::readLaserParameters(const sensor_msgs::LaserScan::ConstPtr &msg) {
    ROS_INFO("reading laser specs");
    beamCount = msg->ranges.size();
    angleStep = (msg->angle_max - msg->angle_min) / (float) beamCount;

    ROS_INFO("Beam count: %zu", beamCount);
    ROS_INFO("Angle step: %f", angleStep);
    ROS_INFO("done");
}

void LaserScanTransceiver::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    beamCount = msg->ranges.size();

    std::vector<std::pair<double, double>> radiusAngles;

    for (int i = 0; i < beamCount; i++) {
        radiusAngles.emplace_back(i * angleStep, msg->ranges[i]);
    }

    double range_step = (msg->range_max - msg->range_min) / (double) RANGE_STEPS;

    std::vector<std::vector<int>> accumulator = hough_space(radiusAngles, msg->angle_min, msg->angle_max,
                                                            msg->range_min, msg->range_max, angleStep, range_step);

//    ros::NodeHandle n;
//
//    ros::Publisher hough_accumulator_publisher = n.advertise<>("/capo_diff_drive_controller/cmd_vel", 1000);
//    ros::Rate loop_rate(10);

//    hough_accumulator_publisher.publish(msg);
}
