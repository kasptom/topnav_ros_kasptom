#ifndef TOPNAV_GAZEBO_HOUGH_LIDAR_H
#define TOPNAV_GAZEBO_HOUGH_LIDAR_H

#include <vector>
#include <models/AngleRange.h>

static double HOUGH_SPACE_THETA_RANGE = 2 * M_PI;

void
update_hough_space_accumulator(const std::vector<AngleRange> &polarCoordinates,
                               std::vector<std::vector<int>> &accumulator,
                               LaserParameters parameters);

std::vector<std::vector<int>>
create_accumulator(LaserParameters parameters);

#endif //TOPNAV_GAZEBO_HOUGH_LIDAR_H
