#include "../../include/Entities/Circle.hpp"

Circle::Circle()
{
    this->initVariables();
}

Circle::Circle(float radius)
{
    this->initVariables();
    this->property.setRadius(radius);
    this->property.setOrigin(sf::Vector2f(radius, radius));
}

Circle::Circle(float radius, sf::Vector2f position)
{
    this->initVariables();
    this->property.setRadius(radius);
    this->property.setOrigin(sf::Vector2f(radius, radius));
    this->property.setPosition(position);
}

void Circle::initVariables()
{
    this->property.setFillColor(sf::Color::White);
    this->inverseMass = this->mass == 0.f ? 100000.f : 1.f / this->mass;
}

void Circle::calcMomentOfInertia()
{
    // mr^2
    this->momentOfInertia = this->mass * (this->property.getRadius() * this->property.getRadius());
}

void Circle::update(float dt)
{
    this->linearEuler(&this->property);
}

void Circle::render(sf::RenderTarget *target)
{
    target->draw(this->property);
}