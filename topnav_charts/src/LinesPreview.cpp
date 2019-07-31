#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <HokuyoUtils.h>
#include "LinesPreview.h"
#include <constants/topic_names.h>
#include <constants/limits.h>

LinesPreview::LinesPreview() {
    sensor_msgs::LaserScan::ConstPtr ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/capo/laser/scan");
    parameters = HokuyoUtils::read_laser_parameters(ptr);

    update_range_circles(HOUGH_DEFAULT_MAX_POINT_RANGE);

    houghAccumulatorSubscriber = handle.subscribe("/capo/laser/hough", 1000,
                                                  &LinesPreview::onHoughSpaceAccumulatorUpdated, this);
    topnavConfigSubscriber = handle.subscribe(TOPIC_NAME_TOPNAV_CONFIG, 1000,
                                              &LinesPreview::on_configuration_message_received, this);

    laserPointsSubscriber = handle.subscribe("/capo/laser/scan", 1000, &LinesPreview::onLaserPointsUpdated, this);
}

std::vector<sf::RectangleShape> LinesPreview::get_lines() {
    return lines;
}

void LinesPreview::onHoughSpaceAccumulatorUpdated(const topnav_msgs::HoughAcc::ConstPtr &msg) {
    lines.clear();

    size_t cols = msg->accumulator[0].acc_row.size();
    size_t rows = msg->accumulator.size();

    int occurrences = 0;

    for (int rho_idx = 0; rho_idx < rows; rho_idx++) {
        for (int theta_idx = 0; theta_idx < cols; theta_idx++) {
            occurrences = msg->accumulator[rho_idx].acc_row[theta_idx];
            if (occurrences >= line_detection_votes_threshold) {
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
        x = (range * std::cos(angle) - msg->range_min) / (msg->range_max - msg->range_min) * LIDAR_PREVIEW_WIDTH / 2;
        y = (range * std::sin(angle) - msg->range_min) / (msg->range_max - msg->range_min) * LIDAR_PREVIEW_HEIGHT / 2;

        sf::RectangleShape point(sf::Vector2f(POINT_SIZE_PX, POINT_SIZE_PX));
        point.setFillColor(sf::Color(255, 0, 0, 255));
        point.setPosition(-y, -x);
        point.move(sf::Vector2f(LIDAR_PREVIEW_WIDTH / 2.0f - POINT_SIZE_PX / 2.0f, LIDAR_PREVIEW_HEIGHT / 2.0f - POINT_SIZE_PX / 2.0f));
        points.push_back(point);
    }
}

/**
 *
 * @param rho_idx accumulator's row indicating the HoughSpace's rho (distance)
 * @param theta_idx accumulator's column indicating the HoughSpace's theta (angle)
 */
void LinesPreview::createLineToDraw(int rho_idx, int theta_idx) {
    sf::RectangleShape line(sf::Vector2f(2, LIDAR_PREVIEW_HEIGHT * 2));

    double rho = parameters.get_range_min() + rho_idx * parameters.get_range_step();
    double theta = theta_idx * parameters.get_angle_step();   // radians

    float max_range = parameters.get_range_max();
    float min_range = parameters.get_range_min();

    sf::RectangleShape rhoLine(sf::Vector2f(2, static_cast<float>(LIDAR_PREVIEW_WIDTH / 2.0f * rho / (max_range - min_range))));
    rhoLine.setFillColor(sf::Color(0, 255, 0));
    rhoLine.setPosition(LIDAR_PREVIEW_WIDTH / 2.0f, LIDAR_PREVIEW_HEIGHT / 2.0f);

    double x_coords = rho * cos(theta);
    double y_coords = rho * sin(theta);

    auto x = static_cast<int>(LIDAR_PREVIEW_WIDTH / 2.0f * x_coords /(max_range - min_range));
    auto y = static_cast<int>(LIDAR_PREVIEW_HEIGHT / 2.0f * y_coords /(max_range - min_range));
    auto rotation = static_cast<int>(theta / M_PI * 180);

    rhoLine.setRotation(- rotation - 90.0f);

    ROS_INFO("(%5.2f, %5.2f) theta=%05.2f, rho=%05.2f, x=%d y=%d", x_coords, y_coords, theta * 180 / M_PI, rho, x , y);

    line.setOrigin(1, LIDAR_PREVIEW_HEIGHT);
    line.setPosition(LIDAR_PREVIEW_WIDTH / 2.0f + x, LIDAR_PREVIEW_HEIGHT / 2.0f - y);
    line.setRotation(-rotation);

    lines.push_back(line);
    lines.push_back(rhoLine);
}

std::vector<sf::RectangleShape> LinesPreview::get_points() {
    return points;
}

sf::CircleShape LinesPreview::get_max_range_circle() {
    return max_range_circle;
}

void LinesPreview::on_configuration_message_received(const topnav_msgs::TopNavConfigMsg &msg) {
    line_detection_votes_threshold = msg.line_detection_threshold;
    ROS_INFO("LinesPreview: line detecion votes threshold changed to: %d", line_detection_votes_threshold);
    update_range_circles(msg.hough_max_point_range);
    ROS_INFO("LinesPreview: max Hough point range to: %.2f", msg.hough_max_point_range);
}

void LinesPreview::update_range_circles(double max_hough_point_range) {
    auto circle_radius = static_cast<float>((max_hough_point_range / HOKUYO_MAX_AVAILABLE_RANGE_METERS) * (LIDAR_PREVIEW_HEIGHT / 2.0));
    max_range_circle = sf::CircleShape(circle_radius);
    max_range_circle.setFillColor(sf::Color::Transparent);
    max_range_circle.setOrigin(circle_radius, circle_radius);
    max_range_circle.setPosition(LIDAR_PREVIEW_WIDTH / 2.0f, LIDAR_PREVIEW_HEIGHT / 2.0f);
    max_range_circle.setOutlineThickness(2);
    max_range_circle.setOutlineColor(sf::Color::Yellow);
}
