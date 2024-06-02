#include "catch.hpp"
#include "../include/Entities/Entities.hpp"

TEST_CASE("Rigidbody Functions")
{
    sf::Vector2f force = sf::Vector2f(5.f, 3.f);
    Circle circle = Circle(10.f);
    circle.property.setPosition(sf::Vector2f(2.f, 3.f));
    circle.mass = 2.f;
    circle.calcMomentOfInertia();
    circle.force = force;

    SECTION("Integration Functions")
    {
        //TODO: linearEuler and Verlet
    }

    SECTION("Angular Motion Functions")
    {
        //test torque
        circle.calcTorque(&circle.property);
        REQUIRE(circle.torque == Approx(-9.f));

        //test angular acceleration
        circle.calcAngularAcceleration(&circle);
        REQUIRE(circle.angularAcceleration == Approx(-0.045));
    }
}