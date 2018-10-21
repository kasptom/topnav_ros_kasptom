#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/LaserScan.h"
#include "ArUcoDetector.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_data_reader");
    ArUcoDetector detector;
    ros::spin();
    return 0;
}

