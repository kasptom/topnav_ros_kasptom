//
// Created by kasptom on 7/14/18.
//

#ifndef TOPNAV_CHARTS_HOUGHLINESPREVIEW_H
#define TOPNAV_CHARTS_HOUGHLINESPREVIEW_H


#include <ros/ros.h>
#include <SFML/Graphics/RectangleShape.hpp>
#include <topnav_msgs/HoughAcc.h>

static const int PREVIEW_WIDTH = 480;
static const int PREVIEW_HEIGHT = 480;

class HoughLinesPreview {
public:
    HoughLinesPreview();

    std::vector<sf::RectangleShape> get_grid();

private:
    ros::Subscriber houghAccumulatorSubscriber;
    ros::NodeHandle handle;
    std::vector<sf::RectangleShape> grid;

    void drawerCallback(const topnav_msgs::HoughAcc::ConstPtr &msg);
};


#endif //TOPNAV_CHARTS_HOUGHLINESPREVIEW_H
