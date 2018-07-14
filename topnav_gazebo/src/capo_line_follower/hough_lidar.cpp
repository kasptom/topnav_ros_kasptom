#include <cmath>
#include <vector>
#include <cmath>
#include <cstdio>
#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "hough_lidar.h"

void print_accumulator(std::vector<std::vector<int>> accumulator);

std::vector<std::vector<int>>
create_accumulator(LaserParameters parameters) {
    float min_range = parameters.get_range_min();
    float max_range = parameters.get_range_max();
    float range_step = parameters.get_range_step();
    float min_angle = parameters.get_angle_min();
    float max_angle = parameters.get_angle_max();
    float angle_step = parameters.get_angle_step();

    auto rangeBucketsCount = static_cast<int>(std::ceil((max_range - min_range) / range_step));
    auto angleBucketsCount = static_cast<int>(std::ceil((max_angle - min_angle) / angle_step));

    std::vector<std::vector<int>> accumulator;

    for (int i = 0; i < rangeBucketsCount; i++) {
        accumulator.emplace_back(angleBucketsCount);
    }

    accumulator[2][3] = 10;

    return accumulator;
}

void
hough_space(const std::vector<std::pair<double, double>> &polarCoordinates, std::vector<std::vector<int>> accumulator,
            LaserParameters parameters) {
    // TODO Hough Transform
//    print_accumulator(accumulator);
}

void print_accumulator(std::vector<std::vector<int>> accumulator) {
    std::stringstream string_stream("", std::ios_base::app | std::ios_base::out);

    ROS_INFO("----------------------------------------------------------------------");
    for (auto row : accumulator) {
        for (int j : row) {
            string_stream << boost::format("%1% ") % j;
        }
        ROS_INFO("%s", string_stream.str().c_str());
        string_stream.str(std::string());
        string_stream.clear();
    }
    ROS_INFO("----------------------------------------------------------------------");
}
