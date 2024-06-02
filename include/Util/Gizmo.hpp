#include "../GameObject.hpp"
#include "../Entities/Entities.hpp"
#include "Math.hpp"

class Gizmo
{
public:
    Line line;
    Circle circle;

public:
    Gizmo();

public:
    void drawAll(sf::RenderTarget *target);
    void drawRay(sf::Vector2f start_position, sf::Vector2f direction);
    void drawCircle(sf::Vector2f position, float radius, sf::Color color);
};