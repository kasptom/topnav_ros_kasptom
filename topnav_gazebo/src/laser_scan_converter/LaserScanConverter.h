#ifndef TOPNAV_GAZEBO_LASERSCANTRANSCEIVER_H
#define TOPNAV_GAZEBO_LASERSCANTRANSCEIVER_H


#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <topnav_msgs/HoughAcc.h>
#include <ros/node_handle.h>
#include <models/LaserParameters.h>
#include <topnav_msgs/AngleRangesMsg.h>
#include <topnav_msgs/TopNavConfigMsg.h>
#include <constants/limits.h>
#include <constants/topic_names.h>
#include <models/AngleRange.h>

static const double NOISE_RANGE = 0.09;

class LaserScanConverter {
public:
    LaserScanConverter();

    void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

    void topnav_config_callback(const topnav_msgs::TopNavConfigMsg::ConstPtr &msg);

private:
    ros::NodeHandle handle;
    ros::Publisher hough_space_publisher;
    ros::Publisher angle_range_lidar_publisher;
    ros::Subscriber laser_scan_subscriber;
    ros::Subscriber topnav_config_subscriber;
    LaserParameters parameters = LaserParameters(0, 0, 0, 0, 0);

    double hough_max_point_range = HOUGH_DEFAULT_MAX_POINT_RANGE;

    topnav_msgs::HoughAcc create_hough_message(std::vector<std::vector<int>> rhoThetaMatrix);

    topnav_msgs::AngleRangesMsg_<std::allocator<void>>
    create_angle_range_message(std::vector<AngleRange> polar_coordinates);

    void filter_out_noise(std::vector<AngleRange> &vector);

    bool is_noise(AngleRange &range);

    bool is_nan(const AngleRange &angle_range) const;

    void filter_out_too_far_points(std::vector<AngleRange> &angleRanges);
};


#endif //TOPNAV_GAZEBO_LASERSCANTRANSCEIVER_H
