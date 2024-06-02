#include "catch.hpp"
#include "../include/Util/Collision.hpp"
#include "../include/Entities/Entities.hpp"

SCENARIO("Overlapping")
{
    Collision collision;

    GIVEN("Two Boxes")
    {
        Box a = Box();
        Box b = Box();

        WHEN("Two Boxes Overlap")
        {
            a.property.setPosition(2.f, 2.f);
            b.property.setPosition(2.f, 2.5f);

            sf::FloatRect a_bound = a.property.getGlobalBounds();
            sf::FloatRect b_bound = b.property.getGlobalBounds();
            bool overlap = collision._boxOverlapping(a_bound.left, a_bound.left + a_bound.width,
                                                     b_bound.left, b_bound.left + b_bound.width) &&
                           collision._boxOverlapping(a_bound.top, a_bound.top + a_bound.height,
                                                     b_bound.top, b_bound.top + b_bound.height);
            THEN("Overlapping flag should be true")
            {
                REQUIRE(overlap == true);
            }
        }
    }

    GIVEN("Two Circles")
    {
        Circle a = Circle(10.f, sf::Vector2f(50.f, 50.f));
        Circle b = Circle(10.f, sf::Vector2f(50.f, 40.f));

        WHEN("Two Circles Overlap")
        {
            bool overlap = collision._circleOverlapping(a.property.getRadius(), b.property.getRadius(),
                                                        a.property.getPosition(), b.property.getPosition());

            THEN("Overlapping flag should be true")
            {
                REQUIRE(overlap == true);
            }
        }
    }

    GIVEN("A Circle and a Line Segment")
    {
        Circle circle = Circle(10.f, sf::Vector2f(55.f, 55.f));
        Line line = Line(sf::Vector2f(50.f, 50.f), sf::Vector2f(50.f, 60.f));
        WHEN("A Circle and a Line Segment overlap")
        {
            bool overlap = collision._circleSegmentOverlapping(circle, line);
            THEN("Overlapping flag should be true")
            {
                REQUIRE(overlap == true);
            }
        }
    }
}

SCENARIO("Collision")
{
    Collision collision;

    GIVEN("Two boxes")
    {
        Box a = Box();
        Box b = Box();
        WHEN("Two Boxes Collide")
        {
            a.property.setPosition(sf::Vector2f(50.f, 50.f));
            b.property.setPosition(sf::Vector2f(50.f, 55.f));

            bool collide = collision._boxCollide(a, b);
            THEN("Collision flag should be true")
            {
                REQUIRE(collide == true);
            }
        }
    }

    GIVEN("Two Circles")
    {
        Circle a = Circle(10.f, sf::Vector2f(50.f, 50.f));
        Circle b = Circle(10.f, sf::Vector2f(50.f, 60.f));
        WHEN("Two Circles Collide")
        {
            bool collide = collision._circleCollide(a, b);
            THEN("Collision flag should be true")
            {
                REQUIRE(collide == true);
            }
        }
    }

    GIVEN("A Circle and a Point")
    {
        Circle circle = Circle(10.f);
        sf::Vector2f point = sf::Vector2f(50.f, 50.f);
        WHEN("A Circle and a Point Collide")
        {
            circle.property.setPosition(sf::Vector2f(60.f, 50.f));
            bool collide = collision._circlePointCollide(circle, point);
            THEN("Collision flag should be true")
            {
                REQUIRE(collide == true);
            }
        }
    }

    GIVEN("A Circle and a Box")
    {
        Circle circle = Circle(10.f);
        Box box = Box();
        WHEN("A Circle and a Box Collide")
        {
            circle.property.setPosition(sf::Vector2f(50.f, 50.f));
            box.property.setPosition(sf::Vector2f(50.f, 60.f));

            bool collide = collision._circleBoxCollide(circle, box);
            THEN("Collision flag should be true")
            {
                REQUIRE(collide == true);
            }
        }
    }

    GIVEN("Oriented Boxes")
    {
        Box a = Box(10, sf::Vector2f(1.f, 5.f));
        Box b = Box(10, sf::Vector2f(4.f, 2.f));

        WHEN("The two boxes collide")
        {
            bool collide = collision._orientedBoxCollide(a, b);
            THEN("Collision flag should be true")
            {
                REQUIRE(collide == true);
            }
        }
    }

    GIVEN("Circle and Oriented Box")
    {
        Circle circle = Circle(2.f, sf::Vector2f(5.f, 7.f));
        Box box = Box(sf::Vector2f(6.f, 2.f), sf::Vector2f(5.f, 4.f));
        box.property.setRotation(30.f);
        WHEN("The two objects collide")
        {
            bool collide = collision._circleOrientedBoxCollide(circle, box);
            THEN("Collision flag should be true")
            {
                REQUIRE(collide == true);
            }
        }
    }

    GIVEN("Box and a Point")
    {
        Box box = Box(sf::Vector2f(10.f, 10.f), sf::Vector2f(20.f, 20.f));
        sf::Vector2f point = sf::Vector2f(24.f, 20.f);
        WHEN("The two object collide")
        {
            bool collide = collision._boxPointCollide(box, point);
            THEN("Collision flag should be true")
            {
                REQUIRE(collide == true);
            }
        }
    }
}