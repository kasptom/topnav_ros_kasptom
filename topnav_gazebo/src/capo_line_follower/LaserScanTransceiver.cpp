#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "LaserScanTransceiver.h"
#include "hough_lidar.h"
#include <topnav_msgs/TestMessage.h>
#include <HokuyoUtils.h>

LaserScanTransceiver::LaserScanTransceiver() {
    sensor_msgs::LaserScan::ConstPtr ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/capo/laser/scan");
    parameters = HokuyoUtils::read_laser_parameters(ptr);
    laser_scan_subscriber = handle.subscribe("/capo/laser/scan", 1000, &LaserScanTransceiver::laser_scan_callback,
                                             this);
    hough_space_publisher = handle.advertise<topnav_msgs::HoughAcc>(TOPIC_NAME_LASER_TRANSCEIVER, 1000);
}

void LaserScanTransceiver::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    std::vector<std::pair<double, double>> anglesRadius;

    for (int i = 0; i < parameters.get_beam_count(); i++) {
        if (msg->ranges[i] == INFINITY) continue;

        anglesRadius.emplace_back(i * parameters.get_angle_step(), msg->ranges[i]);
    }

    std::vector<std::vector<int>> accumulator = create_accumulator(parameters);

    hough_space(anglesRadius, accumulator, parameters);

    topnav_msgs::HoughAcc houghMessage = compose_message(accumulator);
    hough_space_publisher.publish(houghMessage);
}

topnav_msgs::HoughAcc LaserScanTransceiver::compose_message(std::vector<std::vector<int>> vector) {
    topnav_msgs::HoughAcc msg;

    for (int i = 0; i < vector.size(); i++) {
        topnav_msgs::HoughAccRow row;
        for (int j = 0; j < vector[0].size(); j++) {
            row.acc_row.push_back(vector[i][j]);
        }
        msg.accumulator.push_back(row);
    }

    return msg;
}
