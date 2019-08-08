#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/LaserScan.h"
#include "LaserScanConverter.h"
#include <gazebo_msgs/SetModelState.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_data_reader");
    LaserScanConverter converter;
    ros::spin();
    return 0;
}
