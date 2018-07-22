#include "topnav_lines_preview_node.h"
#include "HoughLinesPreview.h"


int main(int argc, char **argv) {


    sf::RenderWindow window(sf::VideoMode(PREVIEW_WIDTH, PREVIEW_HEIGHT), "Hough preview");
    ros::init(argc, argv, "houg_lines_preview");
    HoughLinesPreview preview;

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