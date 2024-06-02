#pragma once

#include <iostream>
#include <SFML/Graphics.hpp>
#include "Math.hpp"
#include "../Entities/Entities.hpp"
#include "../Globals.hpp"

using namespace std;

class Collision
{
public:
    // storing vital positions
    sf::Vector2f nearest_point;
    // intersection check
    bool _boxOverlapping(float min_a, float max_a, float min_b, float max_b);
    bool _circleOverlapping(float radius_a, float radius_b, sf::Vector2f origin_a, sf::Vector2f origin_b);
    bool _boxLineOverlapping();
    bool _circleSegmentOverlapping(Circle &a, Line &l);
    // actual collision function
    bool _linesCollide(Line &a, Line &b);
    bool _boxCollide(Box &a, Box &b);
    bool _circleCollide(Circle &a, Circle &b);
    bool _boxLineCollide();
    bool _boxPointCollide(Box &box, sf::Vector2f point);
    bool _circlePointCollide(Circle &a, sf::Vector2f point);
    bool _circleWindowCollide(Circle &a, Line l1, Line l2, Line l3, Line l4);
    bool _circleBoxCollide(Circle &circle, Box &box);
    bool _orientedBoxCollide(Box &a, Box &b);
    bool _circleOrientedBoxCollide(Circle &circle, Box &box);
    // penetretion and collision resolution
    // circle
    void _circlePenetrationResolution(Circle &a, Circle &b);
    void _circleCollisionResolution(Circle &a, Circle &b);
    void _circlePointPenetrationResolution(Circle &circle, sf::Vector2f &p);
    void _circlePointCollisionResolution(Circle &circle, sf::Vector2f &point);
    // box
    void _boxPenetrationResolution(Box &a, Box &b);
    void _boxCollisionResolution(Box &a, Box &b);
    // circle - box
    void _circleBoxPenetrationResolution(Circle &circle, Box &box);
    void _circleBoxCollisionResolution(Circle &circle, Box &box);
    //...
    sf::Vector2f clampOnRectangle(sf::Vector2f point, Box box);
};