#include <topnav_msgs/HoughAcc.h>
#include "HoughSpacePreview.h"

HoughSpacePreview::HoughSpacePreview() {
    houghAccumulatorSubscriber = handle.subscribe("/capo/laser/hough", 1000, &HoughSpacePreview::drawerCallback, this);
}

void HoughSpacePreview::drawerCallback(const topnav_msgs::HoughAcc::ConstPtr &msg) {
    grid.clear();
    float tile_height = PREVIEW_HEIGHT / (float) msg->accumulator.size();
    float tile_width = PREVIEW_WIDTH / (float) msg->accumulator[0].acc_row.size();

    size_t cols = msg->accumulator[0].acc_row.size();
    size_t rows = msg->accumulator.size();

    int occurences;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            sf::RectangleShape rectangle(sf::Vector2f(tile_width, tile_height));
            occurences = msg->accumulator[i].acc_row[j];

            if (occurences != 0) {
                rectangle.setFillColor(sf::Color(255, 0, 0,
                                                 static_cast<sf::Uint8>(255 * std::min((occurences / 5), 1))));
            }
            else {
                rectangle.setFillColor(sf::Color(255, 255, 255));
            }
            rectangle.setOrigin(-j * tile_width, -i * tile_height);
            grid.push_back(rectangle);
        }
    }

}

std::vector<sf::RectangleShape> HoughSpacePreview::get_grid() {
    return grid;
}
