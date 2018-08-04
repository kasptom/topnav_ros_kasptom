#include "LinesPreview.h"
#include "HoughSpacePreview.h"

LinesPreview::LinesPreview() {
    houghAccumulatorSubscriber = handle.subscribe("/capo/laser/hough", 1000,
                                                  &LinesPreview::onHoughSpaceAccumulatorUpdated, this);
}

std::vector<sf::RectangleShape> LinesPreview::get_grid() {
    return grid;
}

void LinesPreview::onHoughSpaceAccumulatorUpdated(const topnav_msgs::HoughAcc::ConstPtr &msg) {

}

void LinesPreview::showMostFrequentLines(const topnav_msgs::HoughAcc::ConstPtr &msg) {
    // TODO znalezienie w akumulatorze pol o najwiekszej wartosci
    // obliczenie ro i theta na podstawie indeksow
    // przejscie z ro, theta na x, y
    // narysowanie prostych przekraczajacych prog
}
