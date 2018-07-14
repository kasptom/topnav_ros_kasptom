#include <topnav_msgs/HoughAcc.h>
#include "HoughLinesPreview.h"

HoughLinesPreview::HoughLinesPreview() {
    houghAccumulatorSubscriber = handle.subscribe("/capo/laser/hough", 1000, &HoughLinesPreview::drawerCallback, this);
}

void HoughLinesPreview::drawerCallback(const topnav_msgs::HoughAcc::ConstPtr &msg) {
    grid.clear();
    grid.push_back(sf::RectangleShape(sf::Vector2f(10,10)));
    grid[0].setFillColor(sf::Color::Red);
}

std::vector<sf::RectangleShape> HoughLinesPreview::get_grid() {
    return grid;
}
