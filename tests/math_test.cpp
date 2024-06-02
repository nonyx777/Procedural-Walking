#include "catch.hpp"
#include "../include/Util/Math.hpp"

TEST_CASE("Math Tests")
{
    SECTION("Trig Operations")
    {
        REQUIRE(round(Math::_sin(90.f)) == 1.f);
        REQUIRE(round(Math::_cos(90.f)) == 0.f);
    }

    SECTION("Angle Conversion")
    {
        float degree = 90.f;
        float radian = 1.5708f;

        float expected_deg = round(radian * Math::_rad2Deg());
        float expected_rad = degree * Math::_deg2Rad();

        REQUIRE(expected_deg == degree);
        REQUIRE(expected_rad == Approx(radian));
    }

    SECTION("Vector Operations")
    {
        sf::Vector2f vec1 = sf::Vector2f(0.f, 2.f);
        sf::Vector2f vec2 = sf::Vector2f(1.f, 1.f);

        REQUIRE(Math::_length(vec1) == 2);
        REQUIRE(Math::_displacement(vec2, vec1) == sf::Vector2f(1.f, -1.f));
        REQUIRE(Math::_normalize(vec1) == sf::Vector2f(0.f, 1.f));
        REQUIRE(Math::_dot(vec1, vec2) == 2.f);
    }

    SECTION("Utility Operations")
    {
        REQUIRE(Math::_clampOnRange(4.f, 2.f, 3.f) == 3.f);

        GIVEN("An array of vectors")
        {
            std::vector<sf::Vector2f> vectors;
            sf::Vector2f vec1 = sf::Vector2f(4.f, 4.f);
            sf::Vector2f vec2 = sf::Vector2f(6.f, 6.f);
            sf::Vector2f vec3 = sf::Vector2f(1.f, 5.f);

            vectors.push_back(vec1);
            vectors.push_back(vec2);
            vectors.push_back(vec3);

            WHEN("Comparing them with eachother")
            {
                THEN("The minimum should be returned")
                {
                    sf::Vector2f minimum = Math::_minVector(vectors);

                    REQUIRE(minimum == vectors[2]);
                }
            }
        }

        GIVEN("An array of vectors")
        {
            std::vector<sf::Vector2f> vectors;
            sf::Vector2f vec1 = sf::Vector2f(4.f, 4.f);
            sf::Vector2f vec2 = sf::Vector2f(6.f, 6.f);
            sf::Vector2f vec3 = sf::Vector2f(1.f, 5.f);

            vectors.push_back(vec1);
            vectors.push_back(vec2);
            vectors.push_back(vec3);

            WHEN("Comparing them with eachother")
            {
                THEN("The maximum should be returned")
                {
                    sf::Vector2f maximum = Math::_maxVector(vectors);

                    REQUIRE(maximum == vectors[1]);
                }
            }
        }

        REQUIRE(Math::_lerp(2.f, 4.f, 0.5f) == Approx(3.f));
    }
}