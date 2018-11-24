#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "HokuyoUtils.h"
#include "models/AngleRange.h"
#include <vector>

LaserParameters HokuyoUtils::read_laser_parameters(const sensor_msgs::LaserScan::ConstPtr &msg) {
    ROS_INFO("reading laser specs");

    LaserParameters parameters(msg->angle_min, msg->angle_max, msg->range_min, msg->range_max, msg->ranges.size());

    ROS_INFO("Beam count: %zu", parameters.get_beam_count());
    ROS_INFO("Angle step: %f", parameters.get_angle_step());
    ROS_INFO("done");
    return parameters;
}

std::vector<AngleRange>
HokuyoUtils::map_laser_scan_to_range_angle_data(
        const sensor_msgs::LaserScan_<std::allocator<void> >::ConstPtr &msg, LaserParameters parameters) {
    std::vector<AngleRange> polar_coordinates;

    for (int i = 0; i < parameters.get_beam_count(); i++) {
        if (msg->ranges[i] == INFINITY) continue;

        polar_coordinates.push_back(AngleRange(HokuyoUtils::calculate_angle(i, parameters), msg->ranges[i]));
    }
    return polar_coordinates;
}

float HokuyoUtils::calculate_angle(int lidar_angle_index, LaserParameters parameters) {
    return parameters.get_angle_min() + lidar_angle_index * parameters.get_angle_step();
}