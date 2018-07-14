#ifndef TOPNAV_GAZEBO_TOPNAV_CHARTS_H
#define TOPNAV_GAZEBO_TOPNAV_CHARTS_H

#include <iostream>
#include "topnav_lines_preview_node.h"
#include <SFML/Graphics.hpp>
#include <topnav_msgs/HoughAcc.h>

std::vector<sf::RectangleShape> create_grid_from_accumulator(topnav_msgs::HoughAcc accumulatorMessage);

#endif //TOPNAV_GAZEBO_TOPNAV_CHARTS_H
