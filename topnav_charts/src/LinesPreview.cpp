#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <HokuyoUtils.h>
#include "LinesPreview.h"

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

    int lineOccurrencesThreshold = 4;    // TODO to constant
    int occurrences = 0;

    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            occurrences = msg->accumulator[row].acc_row[col];
            if (occurrences >= lineOccurrencesThreshold)
            {
                createLineToDraw(row,col);
            }
        }
    }
}

void LinesPreview::onLaserPointsUpdated(const sensor_msgs::LaserScan::ConstPtr &msg) {
    points.clear();

    float x, y, angle, range;
    for(int i = 0; i < msg->ranges.size(); i++) {
        angle = msg->angle_min + msg->angle_increment * i;
        range = msg->ranges[i];
        x = range * std::cos(angle) / (msg->range_max - msg->range_min) * PREVIEW_WIDTH / 2;
        y = range * std::sin(angle) / (msg->range_max - msg->range_min) * PREVIEW_HEIGHT / 2;

        sf::RectangleShape point(sf::Vector2f(10, 10));
        point.setFillColor(sf::Color(255, 0,0,255));
        point.setPosition(-y,-x);
        point.move(sf::Vector2f(PREVIEW_WIDTH / 2.0f, PREVIEW_HEIGHT / 2.0f));
        points.push_back(point);
    }
}

/**
 *
 * @param row accumulator's row indicating the HoughSpace's rho (distance)
 * @param col accumulator's column indicating the HoughtSpace's theta (angle)
 */
void LinesPreview::createLineToDraw(int row, int col) {
    sf::RectangleShape line(sf::Vector2f(PREVIEW_WIDTH * 2, 3));

    double rho = parameters.get_range_min() + row * parameters.get_range_step();
    double theta = col * parameters.get_angle_step();   // radians

    int x = static_cast<int>(PREVIEW_WIDTH / 2.0f * (rho * sin(theta) - parameters.get_range_min()) / (parameters.get_range_max() - parameters.get_range_min()));
    int y = static_cast<int>(PREVIEW_HEIGHT / 2.0f * (rho * cos(theta) - parameters.get_range_min()) / (parameters.get_range_max() - parameters.get_range_min()));
    int rotation = static_cast<int>(90 - (theta / M_PI * 180));

    line.setPosition(y, x);
    line.setRotation(rotation);
    line.move(sf::Vector2f(PREVIEW_WIDTH / 2.0f, PREVIEW_HEIGHT / 2.0f));
    lines.push_back(line);
}

std::vector<sf::RectangleShape> LinesPreview::get_points() {
    return points;
}
