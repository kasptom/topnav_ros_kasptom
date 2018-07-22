#ifndef TOPNAV_GAZEBO_HOUGH_LIDAR_H
#define TOPNAV_GAZEBO_HOUGH_LIDAR_H

#include <vector>
#include "LaserParameters.h"

void
hough_space(const std::vector<std::pair<double, double>> &polarCoordinates, std::vector<std::vector<int>> &accumulator,
            LaserParameters parameters);

std::vector<std::vector<int>>
create_accumulator(LaserParameters parameters);

#endif //TOPNAV_GAZEBO_HOUGH_LIDAR_H
