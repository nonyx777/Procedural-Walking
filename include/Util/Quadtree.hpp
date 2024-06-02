#pragma once
#include <iostream>
#include <vector>
#include <SFML/Graphics.hpp>
#include "../GameObject.hpp"
#include "../Entities/Entities.hpp"
#include "Collision.hpp"

class Quad
{
public:
    sf::Vector2f topLeft;
    sf::Vector2f botRight;
    Circle *node;
    Box box;
    bool subdivided = false;

    Quad *ne;
    Quad *nw;
    Quad *sw;
    Quad *se;

    Quad();
    Quad(sf::Vector2f topLeft, sf::Vector2f botRight);
    Collision collision;

    void insert(Circle *node);
    std::vector<Circle *> search(sf::Vector2f topLeft, sf::Vector2f botRight);
    void clear();
};