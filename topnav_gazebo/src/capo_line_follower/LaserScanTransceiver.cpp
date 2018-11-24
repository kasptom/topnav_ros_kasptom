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
    std::vector<AngleRange> polarCoordinates = HokuyoUtils::map_laser_scan_to_range_angle_data(msg, parameters);

    std::vector<std::vector<int>> accumulator = create_accumulator(parameters);

    update_hough_space_accumulator(polarCoordinates, accumulator, parameters);

    topnav_msgs::HoughAcc houghMessage = create_hough_message(accumulator);
    hough_space_publisher.publish(houghMessage);
}

topnav_msgs::HoughAcc LaserScanTransceiver::create_hough_message(std::vector<std::vector<int>> rhoThetaMatrix) {
    topnav_msgs::HoughAcc msg;

    for (int i = 0; i < rhoThetaMatrix.size(); i++) {
        topnav_msgs::HoughAccRow row;
        for (int j = 0; j < rhoThetaMatrix[0].size(); j++) {
            row.acc_row.push_back(rhoThetaMatrix[i][j]);
        }
        msg.accumulator.push_back(row);
    }

    return msg;
}
