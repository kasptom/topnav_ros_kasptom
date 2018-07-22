#include <cmath>
#include <vector>
#include <cmath>
#include <cstdio>
#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "hough_lidar.h"

void print_accumulator(std::vector<std::vector<int>> accumulator);

void update_accumulator(std::pair<double, double> point, std::vector<std::vector<int>> &accumulator,
                        LaserParameters params);

std::vector<std::vector<int>>
create_accumulator(LaserParameters parameters) {
    float min_range = parameters.get_range_min();
    float max_range = parameters.get_range_max();
    float range_step = parameters.get_range_step();
    float min_angle = parameters.get_angle_min();
    float max_angle = parameters.get_angle_max();
    float angle_step = parameters.get_angle_step();

    auto rangeBucketsCount = static_cast<int>(std::ceil((max_range - min_range) / range_step));
    auto angleBucketsCount = static_cast<int>(std::ceil(2 * M_PI / angle_step));

    std::vector<std::vector<int>> accumulator;

    for (int i = 0; i < rangeBucketsCount; i++) {
        accumulator.emplace_back(angleBucketsCount);
    }

    return accumulator;
}

void
hough_space(const std::vector<std::pair<double, double>> &polarCoordinates, std::vector<std::vector<int>> &accumulator,
            LaserParameters parameters) {
    for (std::pair<double, double> point : polarCoordinates) {
        std::pair<double, double> xy_point = std::make_pair(point.second * cos(point.first),
                                                            point.second * sin(point.first));
        update_accumulator(xy_point, accumulator, parameters);
//        ROS_INFO("(%f, %f)", point.first, point.second);
//        ROS_INFO("(%f, %f)", xy_point.first, xy_point.second);
    }
    return;
}

/**
 *
 * @param point - laser read 2D coordinates
 * @param accumulator - angle and radius matrix (Hough Space)
 */
void update_accumulator(std::pair<double, double> point, std::vector<std::vector<int>> &accumulator,
                        LaserParameters params) {

    //FIXME
   // return;
    double radius;
    int steps = static_cast<int>(2 * M_PI / params.get_angle_step());
    double angle;
    int radius_index;
    for (int angle_index = 0; angle_index < steps; angle_index++) {
        angle = params.get_angle_step() * angle_index;
        radius = std::abs(point.first * std::cos(angle) + point.second * std::sin(angle));
        radius_index = static_cast<int>(radius / params.get_range_step());
        accumulator[radius_index][angle_index] += 1;
    }
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
