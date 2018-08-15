#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Window/Event.hpp>
#include <thread>
#include <atomic>
#include "LinesPreview.h"

class RectangleParams
{
public:
    float pos_x;
    float pos_y;
    float org_x;
    float org_y;
    int rotation;

    float width, height;

    RectangleParams() {
        height = PREVIEW_HEIGHT / 2.0f;
        width = 10;

        pos_x = PREVIEW_WIDTH / 2.0f;
        pos_y = PREVIEW_HEIGHT / 2.0f;
        org_x = width / 2;  //half of the rectangle height
        org_y = height / 2;
        rotation = 0;
        print_params();
    }

    void print_params()
    {
        printf("position: (%f, %f)\norigin: (%f, %f)\nrotation: %d\n", pos_x, pos_y, org_x, org_y, rotation);
    }
};

std::atomic<bool> isRunning(true);
RectangleParams params;

void draw_rectangle() {
    sf::RenderWindow window(sf::VideoMode(PREVIEW_WIDTH, PREVIEW_HEIGHT), "SFML test");

    sf::RectangleShape rectangleShape(sf::Vector2f(params.width, params.height));


    while (window.isOpen()) {

        rectangleShape.setPosition(params.pos_x, params.pos_y);
        rectangleShape.setOrigin(params.org_x, params.org_y);
        rectangleShape.setRotation(params.rotation);

        sf::Event event{};
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        window.draw(rectangleShape);
        window.display();
        usleep(10000);
        params.rotation++;
        params.rotation = params.rotation % 360;
        rectangleShape.setRotation(params.rotation);
        rectangleShape.setFillColor(sf::Color(0, 255, 0));
    }
    isRunning.exchange(window.isOpen());
}

void read_input() {
    while (isRunning.load())
    {
        std::cout << "Type in position in x, y"<<std::endl;
        std::cin >> params.pos_x;
        std::cin >> params.pos_y;

//        std::cout << "Type in origin in x, y"<<std::endl;
//        std::cin >> params.org_x;
//        std::cin >> params.org_y;
//
//        std::cout << "Type in rotation"<<std::endl;
//        std::cin >> params.rotation;

        params.print_params();
    }
}

int main(int argc, char **argv) {
    std::thread sfmlThread(draw_rectangle);
    std::thread inputThread(read_input);

    sfmlThread.join();
    inputThread.join();

    return 0;
}