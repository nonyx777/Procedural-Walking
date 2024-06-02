#include "../../include/Util/Gizmo.hpp"

Gizmo::Gizmo()
{
}

void Gizmo::drawAll(sf::RenderTarget *target)
{
    this->line.render(target);
    this->circle.render(target);
}

void Gizmo::drawRay(sf::Vector2f start_position, sf::Vector2f direction)
{
    this->line.base = start_position;
    this->line.direction = direction;
}

void Gizmo::drawCircle(sf::Vector2f position, float radius, sf::Color color)
{
    this->circle.property.setRadius(radius);
    this->circle.property.setOrigin(sf::Vector2f(radius, radius));
    this->circle.property.setPosition(position);
    this->circle.property.setFillColor(color);
}