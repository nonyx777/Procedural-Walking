#pragma once
#include <SFML/Graphics.hpp>
#include "./Entities/Entity.hpp"

class Rigidbody
{
public:
    sf::Vector2f linearVelocity;
    sf::Vector2f linearAcceleration;
    float angularVelocity;
    float angularAcceleration;
    sf::Vector2f force;
    float torque;
    float elasticity;

    void linearEuler(sf::Shape *shape);
    void verlet(sf::Shape *shape);
    void angularEuler(sf::Shape *shape, Entity *entity);
    void calcTorque(sf::Shape *shape);
    void calcAngularAcceleration(Entity *entity);

    // verlet related
    sf::Vector2f current, previous;
};