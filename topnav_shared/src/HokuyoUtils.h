#ifndef TOPNAV_CHARTS_HOKUYOUTILS_H
#define TOPNAV_CHARTS_HOKUYOUTILS_H


#include "LaserParameters.h"
#include "sensor_msgs/LaserScan.h"

class HokuyoUtils {
public:
    static LaserParameters read_laser_parameters(const sensor_msgs::LaserScan::ConstPtr &msg);
};


#endif //TOPNAV_CHARTS_HOKUYOUTILS_H
