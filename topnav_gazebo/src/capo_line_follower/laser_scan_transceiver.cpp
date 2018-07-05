#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/LaserScan.h"
#include "hough_lidar.h"

#include <sstream>
#include <gazebo_msgs/SetModelState.h>


size_t beamCount = 0;
float angleStep = 0;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    ROS_INFO("%f", msg->ranges[100]);
    beamCount = msg->ranges.size();

    std::vector<std::pair<double, double>> radiusAngles;

    for (int i = 0; i < beamCount; i++) {
        radiusAngles.emplace_back(i * angleStep, msg->ranges[i]);
    }

    double range_step = (msg->range_max - msg->range_min) / 100;

    //TODO
    hough_space(radiusAngles, msg->range_min, msg->range_max, msg->angle_min, msg->angle_max, angleStep, range_step);
    ROS_INFO("testing ...");
//    ros::NodeHandle n;

//    ros::Publisher reset_pub = n.advertise<std_msgs::Empty>("/capo_diff_drive_controller/cmd_vel", 1000);
//    ros::Rate loop_rate(10);
//
//    reset_pub.publish(msg);
//
//    ros::spinOnce();
}

void readLaserParameters(const sensor_msgs::LaserScan::ConstPtr &msg) {
    ROS_INFO("reading laser specs");
    beamCount = msg->ranges.size();
    angleStep = (msg->angle_max - msg->angle_min) / (float) beamCount;

    ROS_INFO("Beam count: %zu", beamCount);
    ROS_INFO("Angle step: %f", angleStep);
    ROS_INFO("done");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_data_reader");
    ros::NodeHandle handle;

    sensor_msgs::LaserScan::ConstPtr ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/capo/laser/scan");
    readLaserParameters(ptr);

    ros::Subscriber sub = handle.subscribe("/capo/laser/scan", 1000, laserScanCallback);
    ros::spin();
    return 0;
}
