#include <ros/init.h>
#include <SFML/Window/VideoMode.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include "lines_preview_node.h"
#include "LinesPreview.h"

int main(int argc, char **argv) {
    sf::RenderWindow window(sf::VideoMode(LIDAR_PREVIEW_WIDTH, LIDAR_PREVIEW_HEIGHT), "Points & Lines Preview");
    ros::init(argc, argv, "hough_lines_preview");
    LinesPreview preview;


    int loopFrequencyHz = 50;

    ROS_INFO("publishing with %d [Hz] frequency", loopFrequencyHz);

    ros::Rate rate(loopFrequencyHz); // Hz

    while (window.isOpen()) {
        sf::Event event{};
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        window.draw(preview.get_max_range_circle());

        for (const sf::RectangleShape &rectangle : preview.get_lines()) {
            window.draw(rectangle);
        }

        for (const sf::RectangleShape &point : preview.get_points()) {
            window.draw(point);
        }

        window.display();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}