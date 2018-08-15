#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <HokuyoUtils.h>
#include "LinesPreview.h"
#include "../../topnav_gazebo/src/capo_line_follower/hough_lidar.h"

LinesPreview::LinesPreview() {
    sensor_msgs::LaserScan::ConstPtr ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/capo/laser/scan");
    parameters = HokuyoUtils::read_laser_parameters(ptr);

    houghAccumulatorSubscriber = handle.subscribe("/capo/laser/hough", 1000,
                                                  &LinesPreview::onHoughSpaceAccumulatorUpdated, this);
    laserPointsSubscriber = handle.subscribe("/capo/laser/scan", 1000, &LinesPreview::onLaserPointsUpdated, this);
}

std::vector<sf::RectangleShape> LinesPreview::get_lines() {
    return lines;
}

void LinesPreview::onHoughSpaceAccumulatorUpdated(const topnav_msgs::HoughAcc::ConstPtr &msg) {
    lines.clear();

    size_t cols = msg->accumulator[0].acc_row.size();
    size_t rows = msg->accumulator.size();

    int lineOccurrencesThreshold = 5;    // TODO to constant
    int occurrences = 0;

    for (int rho_idx = 0; rho_idx < rows; rho_idx++) {
        for (int theta_idx = 0; theta_idx < cols; theta_idx++) {
            occurrences = msg->accumulator[rho_idx].acc_row[theta_idx];
            if (occurrences >= lineOccurrencesThreshold) {
//                ROS_INFO("detected line: rho=%f, theta=%f",
//                         parameters.get_range_step() * rho_idx,
//                         360 * parameters.get_angle_step() * theta_idx / HOUGH_SPACE_THETA_RANGE);
                createLineToDraw(rho_idx, theta_idx);
            }
        }
    }
}

void LinesPreview::onLaserPointsUpdated(const sensor_msgs::LaserScan::ConstPtr &msg) {
    points.clear();

    float x, y, angle, range;
    for (int i = 0; i < msg->ranges.size(); i++) {
        angle = msg->angle_min + msg->angle_increment * i;
        range = msg->ranges[i];
        x = (range * std::cos(angle) - msg->range_min) / (msg->range_max - msg->range_min) * PREVIEW_WIDTH / 2;
        y = (range * std::sin(angle) - msg->range_min) / (msg->range_max - msg->range_min) * PREVIEW_HEIGHT / 2;

        sf::RectangleShape point(sf::Vector2f(10, 10));
        point.setFillColor(sf::Color(255, 0, 0, 255));
        point.setPosition(-y, -x);
        point.move(sf::Vector2f(PREVIEW_WIDTH / 2.0f, PREVIEW_HEIGHT / 2.0f));
        points.push_back(point);
    }
}

/**
 *
 * @param rho_idx accumulator's row indicating the HoughSpace's rho (distance)
 * @param theta_idx accumulator's column indicating the HoughtSpace's theta (angle)
 */
void LinesPreview::createLineToDraw(int rho_idx, int theta_idx) {
    sf::RectangleShape line(sf::Vector2f(PREVIEW_WIDTH * 2, 2));

    double rho = parameters.get_range_min() + rho_idx * parameters.get_range_step();
    double theta = theta_idx * parameters.get_angle_step();   // radians

    auto x = PREVIEW_WIDTH / 2 + static_cast<int>(PREVIEW_WIDTH / 2.0f * rho * sin(theta) /
                                                  (parameters.get_range_max() - parameters.get_range_min()));
    auto y = PREVIEW_HEIGHT / 2 + static_cast<int>(PREVIEW_HEIGHT / 2.0f * rho * cos(theta) /
                                                   (parameters.get_range_max() - parameters.get_range_min()));
    auto rotation = static_cast<int>(theta / M_PI * 180);

    line.setPosition(PREVIEW_WIDTH - x, PREVIEW_HEIGHT - y);
    line.setRotation(-rotation);
//    ROS_INFO("x=%d y=%d", x, y);

    lines.push_back(line);
}

std::vector<sf::RectangleShape> LinesPreview::get_points() {
    return points;
}
