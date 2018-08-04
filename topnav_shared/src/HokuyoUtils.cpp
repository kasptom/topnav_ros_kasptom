#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "HokuyoUtils.h"

LaserParameters HokuyoUtils::read_laser_parameters(const sensor_msgs::LaserScan::ConstPtr &msg) {
    ROS_INFO("reading laser specs");

    LaserParameters parameters(msg->angle_min, msg->angle_max, msg->range_min, msg->range_max, msg->ranges.size());

    ROS_INFO("Beam count: %zu", parameters.get_beam_count());
    ROS_INFO("Angle step: %f", parameters.get_angle_step());
    ROS_INFO("done");
    return parameters;
}
