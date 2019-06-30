#ifndef TOPNAV_GAZEBO_LASERSCANTRANSCEIVER_H
#define TOPNAV_GAZEBO_LASERSCANTRANSCEIVER_H


#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <topnav_msgs/HoughAcc.h>
#include <ros/node_handle.h>
#include <models/LaserParameters.h>
#include <topnav_msgs/AngleRangesMsg.h>
#include <models/AngleRange.h>

static const std::string TOPIC_NAME_LASER_HOUGH = "/capo/laser/hough"; // NOLINT
static const std::string TOPIC_NAME_LASER_ANGLE_RANGE = "/capo/laser/angle_range"; // NOLINT

static const double NOISE_RANGE = 0.09;

class LaserScanTransceiver {
public:
    LaserScanTransceiver();

    void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

private:
    ros::NodeHandle handle;
    ros::Publisher hough_space_publisher;
    ros::Publisher angle_range_lidar_publisher;
    ros::Subscriber laser_scan_subscriber;
    LaserParameters parameters = LaserParameters(0, 0, 0, 0, 0);

    topnav_msgs::HoughAcc create_hough_message(std::vector<std::vector<int>> rhoThetaMatrix);

    topnav_msgs::AngleRangesMsg_<std::allocator<void>>
    create_angle_range_message(std::vector<AngleRange> polar_coordinates);

    void filter_out_noise(std::vector<AngleRange> &vector);

    bool is_noise(AngleRange &range);

    bool is_nan(const AngleRange &angle_range) const;
};


#endif //TOPNAV_GAZEBO_LASERSCANTRANSCEIVER_H
