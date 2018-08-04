#ifndef TOPNAV_GAZEBO_LINESPREVIEW_H
#define TOPNAV_GAZEBO_LINESPREVIEW_H

static const int PREVIEW_WIDTH = 480;
static const int PREVIEW_HEIGHT = 480;

#include <SFML/Graphics/RectangleShape.hpp>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <topnav_msgs/HoughAcc.h>

class LinesPreview {
public:
    LinesPreview();
    std::vector<sf::RectangleShape> get_grid();
    void onHoughSpaceAccumulatorUpdated(const topnav_msgs::HoughAcc::ConstPtr &msg);

private:
    ros::Subscriber houghAccumulatorSubscriber;
    ros::NodeHandle handle;
    std::vector<sf::RectangleShape> grid;

    void showMostFrequentLines(const topnav_msgs::HoughAcc::ConstPtr &msg);
};


#endif //TOPNAV_GAZEBO_LINESPREVIEW_H
