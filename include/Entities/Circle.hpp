#pragma once

#include "../GameObject.hpp"
#include "../Rigidbody.hpp"
#include "Entity.hpp"

class Circle : public GameObject, public Rigidbody, public Entity
{
public:
    sf::CircleShape property;

private:
    void initVariables();

public:
    Circle();
    Circle(float radius);
    Circle (float radius, sf::Vector2f position);

    void calcMomentOfInertia() override;
    void update(float dt) override;
    void render(sf::RenderTarget *target) override;
};