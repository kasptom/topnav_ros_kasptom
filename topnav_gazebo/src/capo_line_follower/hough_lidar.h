#ifndef TOPNAV_GAZEBO_HOUGH_LIDAR_H
#define TOPNAV_GAZEBO_HOUGH_LIDAR_H

#include <vector>

std::vector<std::vector<int>>
hough_space(const std::vector<std::pair<double, double>> &polarCoordinates, double min_angle, double max_angle,
            double min_range, double max_range, double angle_step, double range_step);

#endif //TOPNAV_GAZEBO_HOUGH_LIDAR_H
