#ifndef TOPNAV_GAZEBO_LASERSCANTRANSCEIVER_H
#define TOPNAV_GAZEBO_LASERSCANTRANSCEIVER_H


#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>

static const int RANGE_STEPS = 10;

class LaserScanTransceiver {
public:
    LaserScanTransceiver();

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

private:
    ros::Publisher houghSpacePublisher;
    ros::Subscriber laserScanSubscriber;
    size_t beamCount = 0;
    float angleStep = 0;

    /**
     * Call it in the constructor
     */
    void readLaserParameters(const sensor_msgs::LaserScan::ConstPtr &msg);
};


#endif //TOPNAV_GAZEBO_LASERSCANTRANSCEIVER_H
