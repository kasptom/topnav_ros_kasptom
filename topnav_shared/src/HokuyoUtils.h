#ifndef TOPNAV_CHARTS_HOKUYOUTILS_H
#define TOPNAV_CHARTS_HOKUYOUTILS_H


#include "LaserParameters.h"
#include "sensor_msgs/LaserScan.h"

class HokuyoUtils {
public:
    static LaserParameters read_laser_parameters(const sensor_msgs::LaserScan::ConstPtr &msg);

    static float calculate_angle(int lidar_angle_index, LaserParameters parameters);

    static std::vector< std::pair<double, double> >
    map_laser_scan_to_range_angle_data(const sensor_msgs::LaserScan_< std::allocator<void> >::ConstPtr &msg,
                                       LaserParameters parameters);
};


#endif //TOPNAV_CHARTS_HOKUYOUTILS_H
