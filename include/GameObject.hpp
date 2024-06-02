#pragma once
#include <SFML/Graphics.hpp>

class GameObject
{
public:
    virtual void update(float dt) = 0;
    virtual void render(sf::RenderTarget* target) = 0;
};