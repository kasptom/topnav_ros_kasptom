#ifndef TOPNAV_GAZEBO_HOUGH_LIDAR_H
#define TOPNAV_GAZEBO_HOUGH_LIDAR_H

#include <vector>

int **hough_space(std::vector<std::pair<double, double>> polarCoordinates, double min_range, double max_range,
                  double min_angle, double max_angle, double range_step, double angle_step);

#endif //TOPNAV_GAZEBO_HOUGH_LIDAR_H
