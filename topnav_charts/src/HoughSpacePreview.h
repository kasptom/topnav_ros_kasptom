#ifndef TOPNAV_CHARTS_HOUGHLINESPREVIEW_H
#define TOPNAV_CHARTS_HOUGHLINESPREVIEW_H


#include <ros/ros.h>
#include <SFML/Graphics/RectangleShape.hpp>
#include <topnav_msgs/HoughAcc.h>
#include <constants/topic_names.h>

static const int LIDAR_PREVIEW_WIDTH = 480;
static const int LIDAR_PREVIEW_HEIGHT = 480;

class HoughSpacePreview {
public:
    HoughSpacePreview();

    std::vector<sf::RectangleShape> get_grid();

    void onHoughSpaceAccumulatorUpdated(const topnav_msgs::HoughAcc::ConstPtr &msg);

private:
    ros::Subscriber houghAccumulatorSubscriber;
    ros::NodeHandle handle;
    std::vector<sf::RectangleShape> grid;

    void markNumberOfLineOccurrences(float tile_height, float tile_width, int occurences, int houghSpaceRow,
                                     int houghSpaceColumn,
                                     sf::RectangleShape &rectangle) const;
};


#endif //TOPNAV_CHARTS_HOUGHLINESPREVIEW_H
