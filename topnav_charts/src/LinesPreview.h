#ifndef TOPNAV_GAZEBO_LINESPREVIEW_H
#define TOPNAV_GAZEBO_LINESPREVIEW_H

static const int LIDAR_PREVIEW_WIDTH = 480;
static const int LIDAR_PREVIEW_HEIGHT = 480;
static const int POINT_SIZE_PX = 10;

#include <SFML/Graphics/RectangleShape.hpp>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <topnav_msgs/HoughAcc.h>
#include <topnav_msgs/TopNavConfigMsg.h>
#include "../../topnav_shared/src/models/LaserParameters.h"
#include <sensor_msgs/LaserScan.h>

class LinesPreview {
public:
    LinesPreview();

    std::vector<sf::RectangleShape> get_lines();
    std::vector<sf::RectangleShape> get_points();

    void onHoughSpaceAccumulatorUpdated(const topnav_msgs::HoughAcc::ConstPtr &msg);

    void on_configuration_message_received(const topnav_msgs::TopNavConfigMsg &msg);

    void onLaserPointsUpdated(const sensor_msgs::LaserScan::ConstPtr &msg);

private:
    ros::Subscriber houghAccumulatorSubscriber;
    ros::Subscriber topnavConfigSubscriber;
    ros::Subscriber laserPointsSubscriber;
    ros::NodeHandle handle;
    std::vector<sf::RectangleShape> lines;
    std::vector<sf::RectangleShape> points;

    int line_detection_votes_threshold = 5;

private:

    void createLineToDraw(int rho_idx, int theta_idx);

    LaserParameters parameters = {0, 0, 0, 0, 0};
};


#endif //TOPNAV_GAZEBO_LINESPREVIEW_H
