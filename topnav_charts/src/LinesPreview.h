#ifndef TOPNAV_GAZEBO_LINESPREVIEW_H
#define TOPNAV_GAZEBO_LINESPREVIEW_H

static const int PREVIEW_WIDTH = 480;
static const int PREVIEW_HEIGHT = 480;

#include <SFML/Graphics/RectangleShape.hpp>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <topnav_msgs/HoughAcc.h>
#include <LaserParameters.h>

class LinesPreview {
public:
    LinesPreview();

    std::vector<sf::RectangleShape> get_lines();

    void onHoughSpaceAccumulatorUpdated(const topnav_msgs::HoughAcc::ConstPtr &msg);

private:
    ros::Subscriber houghAccumulatorSubscriber;
    ros::NodeHandle handle;
    std::vector<sf::RectangleShape> lines;

    void createLineToDraw(int row, int col);

    LaserParameters parameters = {0, 0, 0, 0, 0};
};


#endif //TOPNAV_GAZEBO_LINESPREVIEW_H
