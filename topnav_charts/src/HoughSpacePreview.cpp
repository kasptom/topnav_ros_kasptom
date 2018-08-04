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

    int occurrences = 0;
    int maxOccurrences = 0;
    double avgOccurrences = 0;
    int nonZeroCounter = 0;
    for (int row = 0; row < rows; row++) {
        for (int column = 0; column < cols; column++) {
            sf::RectangleShape rectangle(sf::Vector2f(tile_width, tile_height));

            if (occurrences > maxOccurrences) maxOccurrences = occurrences;
            avgOccurrences += occurrences;
            if (occurrences != 0) nonZeroCounter++;

            occurrences = msg->accumulator[row].acc_row[column];
            markNumberOfLineOccurrences(tile_height, tile_width, occurrences, row, column, rectangle);
            grid.push_back(rectangle);
        }
    }

    if (nonZeroCounter == 0) nonZeroCounter++;
//    ROS_INFO("MAX: %d, AVG: %.2f, NON_ZERO: %d", maxOccurrences, avgOccurrences / nonZeroCounter, nonZeroCounter);

}

void HoughSpacePreview::markNumberOfLineOccurrences(float tile_height, float tile_width, int occurences,
                                                    int houghSpaceRow, int houghSpaceColumn,
                                                    sf::RectangleShape &rectangle) const {
    if (occurences != 0) {
        sf::Uint8 alpha = static_cast<sf::Uint8>(255 * std::min((100.0 * occurences / 10), 100.0));

        rectangle.setFillColor(
                sf::Color(255, 0, 0, alpha)
        );
    } else {
        rectangle.setFillColor(sf::Color(255, 255, 255));
    }
    rectangle.setOrigin(-houghSpaceColumn * tile_width, -houghSpaceRow * tile_height);
}

std::vector<sf::RectangleShape> HoughSpacePreview::get_grid() {
    return grid;
}
