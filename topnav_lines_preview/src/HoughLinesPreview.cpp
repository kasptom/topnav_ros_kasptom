#include <topnav_msgs/HoughAcc.h>
#include "HoughLinesPreview.h"

HoughLinesPreview::HoughLinesPreview() {
    houghAccumulatorSubscriber = handle.subscribe("/capo/laser/hough", 1000, &HoughLinesPreview::drawerCallback, this);
}

void HoughLinesPreview::drawerCallback(const topnav_msgs::HoughAcc::ConstPtr &msg) {
    grid.clear();
    float tile_height = PREVIEW_HEIGHT / (float) msg->accumulator.size();
    float tile_width = PREVIEW_WIDTH / (float) msg->accumulator[0].acc_row.size();

    size_t cols = msg->accumulator[0].acc_row.size();
    size_t rows = msg->accumulator.size();

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            sf::RectangleShape rectangle(sf::Vector2f(tile_width, tile_height));
            if (msg->accumulator[i].acc_row[j] != 0) {
                rectangle.setFillColor(sf::Color(255, 0, 0));
            }
            rectangle.setOrigin(-j * tile_width, -i * tile_height);
            grid.push_back(rectangle);
        }
    }

}

std::vector<sf::RectangleShape> HoughLinesPreview::get_grid() {
    return grid;
}
