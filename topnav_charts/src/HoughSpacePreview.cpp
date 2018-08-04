#include <topnav_msgs/HoughAcc.h>
#include "HoughSpacePreview.h"

HoughSpacePreview::HoughSpacePreview() {
    houghAccumulatorSubscriber = handle.subscribe("/capo/laser/hough", 1000,
                                                  &HoughSpacePreview::onHoughSpaceAccumulatorUpdated, this);
}

void HoughSpacePreview::onHoughSpaceAccumulatorUpdated(const topnav_msgs::HoughAcc::ConstPtr &msg) {
    grid.clear();
    float tile_height = PREVIEW_HEIGHT / (float) msg->accumulator.size();
    float tile_width = PREVIEW_WIDTH / (float) msg->accumulator[0].acc_row.size();

    size_t cols = msg->accumulator[0].acc_row.size();
    size_t rows = msg->accumulator.size();

    int occurences;
    for (int row = 0; row < rows; row++) {
        for (int column = 0; column < cols; column++) {
            sf::RectangleShape rectangle(sf::Vector2f(tile_width, tile_height));
            occurences = msg->accumulator[row].acc_row[column];
            markNumberOfLineOccurrences(tile_height, tile_width, occurences, row, column, rectangle);
            grid.push_back(rectangle);
        }
    }

}

void HoughSpacePreview::markNumberOfLineOccurrences(float tile_height, float tile_width, int occurences,
                                                    int houghSpaceRow, int houghSpaceColumn,
                                                    sf::RectangleShape &rectangle) const {
    if (occurences != 0) {
        rectangle.setFillColor(
                sf::Color(255, 0, 0, static_cast<sf::Uint8>(255 * std::min((occurences / 5), 1)))
        );
    } else {
        rectangle.setFillColor(sf::Color(255, 255, 255));
    }
    rectangle.setOrigin(-houghSpaceColumn * tile_width, -houghSpaceRow * tile_height);
}

std::vector<sf::RectangleShape> HoughSpacePreview::get_grid() {
    return grid;
}
