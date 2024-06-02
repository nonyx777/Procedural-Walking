#pragma once

#include <iostream>
#include <SFML/Graphics.hpp>
#include <math.h>

#define PI 3.14159265

using namespace std;

class Math
{
public:
    // trig operations
    static float _sin(float degree);
    static float _cos(float degree);
    static float _acos(float point);
    static float _atan2(float y, float x);

    // angle conversion operations
    static float _rad2Deg();
    static float _deg2Rad();

    // vector operations
    static float _length(sf::Vector2f vector);
    static sf::Vector2f _displacement(sf::Vector2f target_position, sf::Vector2f self_position);
    static sf::Vector2f _normalize(sf::Vector2f _displacement);
    static sf::Vector2f _rotate(sf::Vector2f vector, float angle);
    static sf::Vector2f _rotateBy90(sf::Vector2f vector);
    static sf::Vector2f _translate(sf::Vector2f vector_a, sf::Vector2f vector_b, float speed);
    static sf::Vector2f _transalte(sf::Vector2f vector_a, sf::Vector2f vector_b);
    static float _dot(sf::Vector2f a, sf::Vector2f b);
    static bool _parallel(sf::Vector2f a, sf::Vector2f b);
    static sf::Vector2f _project(sf::Vector2f a, sf::Vector2f b);
    static float _angleBtn(sf::Vector2f a, sf::Vector2f b);

    // additional
    static float _clampOnRange(float x, float min, float max);
    static sf::Vector2f _minVector(std::vector<sf::Vector2f> &vectors);
    static sf::Vector2f _maxVector(std::vector<sf::Vector2f> &vectors);
    static float _lerp(float min, float max, float t);
    static float _map(float value, float max1, float max2);
};