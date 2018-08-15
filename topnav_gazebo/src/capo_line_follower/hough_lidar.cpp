#include <cmath>
#include <vector>
#include <cmath>
#include <cstdio>
#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include "hough_lidar.h"

void print_accumulator(std::vector <std::vector<int>> accumulator);

void update_accumulator(std::pair<double, double> xy_pair, std::vector<std::vector<int>> &accumulator,
                        LaserParameters params);

bool isNaN(double radius);

double calculateRho(const std::pair<double, double> &xy_pair, double theta);

std::vector <std::vector<int>>
create_accumulator(LaserParameters parameters) {
    float min_range = parameters.get_range_min();
    float max_range = parameters.get_range_max();
    float range_step = parameters.get_range_step();
    float angle_step = parameters.get_angle_step();

    auto rangeBucketsCount = static_cast<int>(std::ceil((max_range - min_range) / range_step));
    auto angleBucketsCount = static_cast<int>(std::ceil(M_PI / angle_step));

    std::vector <std::vector<int>> accumulator;

    for (int i = 0; i < rangeBucketsCount; i++) {
        accumulator.emplace_back(angleBucketsCount);
    }

    return accumulator;
}

void
update_hough_space_accumulator(const std::vector<std::pair<double, double>> &polarCoordinates,
                               std::vector<std::vector<int>> &accumulator,
                               LaserParameters parameters) {
    for (std::pair<double, double> polarCoord : polarCoordinates) {
        std::pair<double, double> xy_point = std::make_pair(polarCoord.first * cos(polarCoord.second),
                                                            polarCoord.first * sin(polarCoord.second));
        update_accumulator(xy_point, accumulator, parameters);
//        ROS_INFO("(%f, %f)", polarCoord.first, polarCoord.second);
//        ROS_INFO("(%f, %f)", xy_point.first, xy_point.second);
    }
}

/**
 *
 * @param xy_pair - laser read 2D coordinates
 * @param accumulator - angle and radius matrix (Hough Space)
 */
void update_accumulator(std::pair<double, double> xy_pair, std::vector <std::vector<int>> &accumulator,
                        LaserParameters params) {
    double rho;
    auto steps = static_cast<int>(M_PI / params.get_angle_step());
    double theta;
    int rho_idx;

    /*
     * (x,y) point gives his (rho, theta) "votes" to the accumulator
     */
    for (int theta_idx = 0; theta_idx < steps; theta_idx++) {
        theta = params.get_angle_step() * theta_idx;
        rho = calculateRho(xy_pair, theta);

        if (isNaN(rho) || rho == INFINITY || rho < params.get_range_min()) continue;

        rho_idx = static_cast<int>(rho / params.get_range_step());
        accumulator[rho_idx][theta_idx] += 1;
    }
}

double calculateRho(const std::pair<double, double> &xy_pair, double theta) {
    return xy_pair.first * cos(theta) + xy_pair.second * sin(theta);
}

bool isNaN(double radius) { return radius != radius; }

void print_accumulator(std::vector <std::vector<int>> accumulator) {
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
