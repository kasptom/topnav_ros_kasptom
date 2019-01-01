#ifndef TOPNAV_CHARTS_HOKUYOUTILS_H
#define TOPNAV_CHARTS_HOKUYOUTILS_H


#include "models/LaserParameters.h"
#include "sensor_msgs/LaserScan.h"
#include "models/AngleRange.h"

class HokuyoUtils {
public:
    static LaserParameters read_laser_parameters(const sensor_msgs::LaserScan::ConstPtr &msg);

    static float calculate_angle(int lidar_angle_index, LaserParameters parameters);

    /**
     * @param msg
     * @param parameters
     * @return coordinates
     */
    static std::vector<AngleRange>
    map_laser_scan_to_range_angle_data(const sensor_msgs::LaserScan_<std::allocator<void> >::ConstPtr &msg,
                                       LaserParameters parameters);
};


#endif //TOPNAV_CHARTS_HOKUYOUTILS_H
