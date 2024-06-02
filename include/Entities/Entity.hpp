#pragma once
#include <iostream>

class Entity
{
public:
    float mass;
    float inverseMass;
    float momentOfInertia;

    virtual void calcMomentOfInertia() = 0;
};