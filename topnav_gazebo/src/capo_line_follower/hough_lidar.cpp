#include <vector>
#include <cmath>
#include <cstdio>
#include <rosconsole/macros_generated.h>
#include <ros/ros.h>

void print_accumulator(std::vector<std::vector<int>> accumulator);

std::vector<std::vector<int>>
create_accumulator(double min_angle, double max_angle, double min_range, double max_range, double range_step,
                   double angle_step) {
    auto rangeBucketsCount = static_cast<int>(ceil((max_range - min_range) / range_step));
    auto angleBucketsCount = static_cast<int>(ceil((max_angle - min_angle) / angle_step));

    std::vector<std::vector<int>> accumulator;

    for (int i = 0; i < rangeBucketsCount; i++) {
        accumulator.emplace_back(angleBucketsCount);
    }

    return accumulator;
}

int **hough_space(std::vector<std::pair<double, double>> polarCoordinates, double min_range, double max_range,
                  double min_angle, double max_angle, double range_step, double angle_step) {
    std::vector<std::vector<int>> accumulator = create_accumulator(min_angle, max_angle, min_range, max_range,
                                                                   range_step, angle_step);

    print_accumulator(accumulator);
}

void print_accumulator(std::vector<std::vector<int>> accumulator) {
    for (auto row : accumulator) {
        for (int j : row) {
            ROS_INFO("%d ", j);
        }
        ROS_INFO("\n");
    }
}
