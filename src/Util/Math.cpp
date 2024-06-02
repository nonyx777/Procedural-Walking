#include "../../include/Util/Math.hpp"

// defining trig functions
float Math::_sin(float degree)
{
    float radian = degree * Math::_deg2Rad();
    return sin(radian);
}
float Math::_cos(float degree)
{
    float radian = degree * Math::_deg2Rad();
    return cos(radian);
}
float Math::_acos(float point)
{
    double point_double = point;
    float angle_value = acos(point_double) * Math::_rad2Deg();
    return angle_value;
}
float Math::_atan2(float y, float x)
{
    float atan2_value = atan2(y, x) * Math::_rad2Deg();
    return atan2_value;
}
// angle conversion functions
float Math::_rad2Deg()
{
    return 180.f / PI;
}
float Math::_deg2Rad()
{
    return PI / 180.f;
}

// defining vector functions
float Math::_length(sf::Vector2f vector)
{
    return sqrt(pow(vector.x, 2) + pow(vector.y, 2));
}
sf::Vector2f Math::_displacement(sf::Vector2f target_position, sf::Vector2f self_position)
{
    sf::Vector2f displacement = target_position - self_position;
    return displacement;
}
sf::Vector2f Math::_normalize(sf::Vector2f vector)
{
    float length = Math::_length(vector);
    return sf::Vector2f(vector.x, vector.y) / length;
}
sf::Vector2f Math::_rotate(sf::Vector2f vector, float angle)
{
    sf::Vector2f a = sf::Vector2f(Math::_cos(angle), Math::_sin(angle));
    sf::Vector2f b = sf::Vector2f(-Math::_sin(angle), Math::_cos(angle));
    sf::Vector2f rotation = sf::Vector2f(vector.x * a.x - vector.y * b.x,
                                         vector.x * a.y + vector.y * b.y);
    return rotation;
}
sf::Vector2f Math::_rotateBy90(sf::Vector2f vector)
{
    float x = vector.y;
    float y = -vector.x;
    return sf::Vector2f(x, y);
}
sf::Vector2f Math::_translate(sf::Vector2f vector_a, sf::Vector2f vector_b, float speed)
{
    sf::Vector2f distance = Math::_displacement(vector_b, vector_a);
    return vector_a += distance * speed;
}
sf::Vector2f Math::_transalte(sf::Vector2f vector_a, sf::Vector2f _Vector_b)
{
    sf::Vector2f distance = Math::_displacement(_Vector_b, vector_a);
    return vector_a += distance;
}
float Math::_dot(sf::Vector2f a, sf::Vector2f b)
{
    return a.x * b.x + a.y * b.y;
}
bool Math::_parallel(sf::Vector2f a, sf::Vector2f b)
{
    sf::Vector2f rotated_vector_a = Math::_rotateBy90(a);
    float dot_product = Math::_dot(rotated_vector_a, b);

    return (round(dot_product) >= -0.f && round(dot_product) <= 0.f);
}
sf::Vector2f Math::_project(sf::Vector2f a, sf::Vector2f b)
{
    float p_1 = Math::_dot(a, b) / pow(Math::_length(b), 2);
    return b * p_1;
}
float Math::_angleBtn(sf::Vector2f a, sf::Vector2f b)
{
    float angle = Math::_dot(a, b) / Math::_length(a) * Math::_length(b);
    return angle;
}

// defining additional support functions
float Math::_clampOnRange(float x, float min, float max)
{
    if (x < min)
        return min;
    else if (max < x)
        return max;
    else
        return x;
}

sf::Vector2f Math::_minVector(std::vector<sf::Vector2f> &vectors)
{
    sf::Vector2f min_vector = vectors[0];
    float min_value = Math::_dot(min_vector, min_vector);
    for (int i = 1; i < vectors.size(); i++)
    {
        float candidate_min_value = Math::_dot(vectors[i], vectors[i]);
        if (candidate_min_value <= min_value)
        {
            min_vector = vectors[i];
        }
    }

    return min_vector;
}

sf::Vector2f Math::_maxVector(std::vector<sf::Vector2f> &vectors)
{
    sf::Vector2f max_vector = vectors[0];
    float max_value = Math::_dot(max_vector, max_vector);
    for (int i = 1; i < vectors.size(); i++)
    {
        float candidate_max_value = Math::_dot(vectors[i], vectors[i]);
        if (candidate_max_value >= max_value)
        {
            max_vector = vectors[i];
        }
    }

    return max_vector;
}

float Math::_lerp(float min, float max, float t)
{
    float value = (1.f - t) * min + max * t;
    return value;
}

float Math::_map(float value, float max1, float max2)
{
    float result = (value * max2) / max1;
    return result;
}