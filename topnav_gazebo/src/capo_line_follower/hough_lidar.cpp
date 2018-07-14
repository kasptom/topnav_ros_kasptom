#include <vector>
#include <cmath>
#include <cstdio>
#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "hough_lidar.h"

void print_accumulator(std::vector<std::vector<int>> accumulator);

std::vector<std::vector<int>>
create_accumulator(double min_angle, double max_angle, double min_range, double max_range, double angle_step,
                   double range_step) {
    auto rangeBucketsCount = static_cast<int>(ceil((max_range - min_range) / range_step));
    auto angleBucketsCount = static_cast<int>(ceil((max_angle - min_angle) / angle_step));

    std::vector<std::vector<int>> accumulator;

    for (int i = 0; i < rangeBucketsCount; i++) {
        accumulator.emplace_back(angleBucketsCount);
    }

    accumulator[2][3] = 10;

    return accumulator;
}

std::vector<std::vector<int>> hough_space(const std::vector<std::pair<double, double>> &polarCoordinates, double min_angle, double max_angle,
                  double min_range, double max_range, double angle_step, double range_step) {
    std::vector<std::vector<int>> accumulator = create_accumulator(min_angle, max_angle, min_range, max_range,
                                                                   angle_step, range_step);

    print_accumulator(accumulator);

    return accumulator;
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
