#include "hough_space_preview_node.h"
#include "HoughSpacePreview.h"


int main(int argc, char **argv) {


    sf::RenderWindow window(sf::VideoMode(PREVIEW_WIDTH, PREVIEW_HEIGHT), "Hough Space Preview");
    ros::init(argc, argv, "houg_lines_preview");
    HoughSpacePreview preview;

    while (window.isOpen()) {
        sf::Event event{};
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();

        for (const sf::RectangleShape &rectangle : preview.get_grid()) {
            window.draw(rectangle);
        }

        window.display();
        ros::spinOnce();
    }

    return 0;
}