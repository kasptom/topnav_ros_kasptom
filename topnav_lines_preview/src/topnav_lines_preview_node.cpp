#include "topnav_lines_preview_node.h"
#include "HoughLinesPreview.h"


int main(int argc, char **argv) {


    sf::RenderWindow window(sf::VideoMode(480, 480), "SFML works!");
    ros::init(argc, argv, "houg_lines_preview");
    HoughLinesPreview preview;

//    sf::CircleShape shape(100.f);
//    sf::RectangleShape rectangleShape(sf::Vector2f(100, 200));
//    rectangleShape.setOrigin(-460,-460);
//    rectangleShape.setOutlineColor(sf::Color::Yellow);
//
//    shape.setFillColor(sf::Color::Green);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();

        for (sf::RectangleShape rectangle : preview.get_grid()) {
            window.draw(rectangle);
        }

        ros::spinOnce();
        window.display();
    }

    return 0;
}