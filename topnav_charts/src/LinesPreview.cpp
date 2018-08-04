#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include "LinesPreview.h"
//#include <topnav_utils/HokuyoUtils.h> FIXME

LinesPreview::LinesPreview() {
    sensor_msgs::LaserScan::ConstPtr ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/capo/laser/scan");
//    parameters = HokuyoUtils::read_laser_parameters(ptr); FIXME

    houghAccumulatorSubscriber = handle.subscribe("/capo/laser/hough", 1000,
                                                  &LinesPreview::onHoughSpaceAccumulatorUpdated, this);
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



/**
 *
 * @param row accumulator's row indicating the HoughSpace's rho (distance)
 * @param col accumulator's column indicating the HoughtSpace's theta (angle)
 */
void LinesPreview::createLineToDraw(int row, int col) {

}
