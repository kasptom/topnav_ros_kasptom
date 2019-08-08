#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "LaserScanConverter.h"
#include "hough_lidar.h"
#include <topnav_msgs/TestMessage.h>
#include <topnav_msgs/AngleRangesMsg.h>
#include <HokuyoUtils.h>
#include <constants/topic_names.h>

LaserScanConverter::LaserScanConverter() {
    sensor_msgs::LaserScan::ConstPtr ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>(TOPIC_NAME_CAPO_LASER_SCAN);
    parameters = HokuyoUtils::read_laser_parameters(ptr);
    laser_scan_subscriber = handle.subscribe(TOPIC_NAME_CAPO_LASER_SCAN, 1000, &LaserScanConverter::laser_scan_callback,
                                             this);
    topnav_config_subscriber = handle.subscribe(TOPIC_NAME_TOPNAV_CONFIG, 1000,
                                                &LaserScanConverter::topnav_config_callback, this);

    hough_space_publisher = handle.advertise<topnav_msgs::HoughAcc>(TOPIC_NAME_CAPO_LASER_HOUGH, 1000);
    angle_range_lidar_publisher = handle.advertise<topnav_msgs::AngleRangesMsg>(TOPIC_NAME_LASER_ANGLE_RANGE, 1000);
}

void LaserScanConverter::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    std::vector<AngleRange> polarCoordinates = HokuyoUtils::map_laser_scan_to_range_angle_data(msg, parameters);

    filter_out_noise(polarCoordinates);
    filter_out_too_far_points(polarCoordinates);

    std::vector<std::vector<int>> accumulator = create_accumulator(parameters);

    update_hough_space_accumulator(polarCoordinates, accumulator, parameters);

    topnav_msgs::AngleRangesMsg angle_range_message = create_angle_range_message(polarCoordinates);
    angle_range_lidar_publisher.publish(angle_range_message);

    topnav_msgs::HoughAcc hough_message = create_hough_message(accumulator);
    hough_space_publisher.publish(hough_message);
}

void LaserScanConverter::topnav_config_callback(const topnav_msgs::TopNavConfigMsg::ConstPtr &msg) {
    ROS_INFO("LaserScanConverter: changing hough_max_point_range to: %.2f[m]", msg->hough_max_point_range);
    hough_max_point_range = msg->hough_max_point_range;
}

void LaserScanConverter::filter_out_noise(std::vector<AngleRange> &angle_ranges) {
    for (auto &angle_range : angle_ranges) {
        if (is_noise(angle_range)) {
            angle_range.set_range(NAN);
        }
    }
}

void LaserScanConverter::filter_out_too_far_points(std::vector<AngleRange> &angleRanges) {
    for (auto &angle_range: angleRanges) {
        if (angle_range.get_range() > hough_max_point_range) {
            angle_range.set_range(NAN);
        }
    }
}

topnav_msgs::HoughAcc LaserScanConverter::create_hough_message(std::vector<std::vector<int>> rhoThetaMatrix) {
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

topnav_msgs::AngleRangesMsg_<std::allocator<void>>
LaserScanConverter::create_angle_range_message(std::vector<AngleRange> polar_coordinates) {
    topnav_msgs::AngleRangesMsg msg;

    for (auto &polar_coordinate : polar_coordinates) {
        msg.angles.push_back(polar_coordinate.get_angle());
        msg.distances.push_back(polar_coordinate.get_range());
    }

    return msg;
}

bool LaserScanConverter::is_noise(AngleRange &angle_range) {
    return is_nan(angle_range) || angle_range.get_range() < NOISE_RANGE;
}

bool LaserScanConverter::is_nan(const AngleRange &angle_range) const { return angle_range.get_range() != angle_range.get_range(); }
