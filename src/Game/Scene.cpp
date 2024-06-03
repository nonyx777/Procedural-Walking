#include "../../include/Game/Scene.hpp"

Scene *Scene::instance = nullptr;

Scene::Scene()
{
    // setting up joints
    Circle joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f - 30.f, GLOBAL::window_height / 2.f));
    this->joints.push_back(joint);
    joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f, GLOBAL::window_height / 2.f + 50.f));
    this->joints.push_back(joint);
    joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f, GLOBAL::window_height / 2.f + 120.f));
    this->joints.push_back(joint);

    this->upperarm_length = Math::_length(joints[1].property.getPosition() - joints[0].property.getPosition());
    this->forearm_length = Math::_length(joints[2].property.getPosition() - joints[1].property.getPosition());

    // setting up the links
    for (int i = 0; i < this->joints.size() - 1; i++)
    {
        Line line = Line(this->joints[i].property.getPosition(), this->joints[i + 1].property.getPosition());
        this->links.push_back(line);
    }

    // target...
    target = Circle(10.f, joints[2].property.getPosition() - sf::Vector2f(50.f, 100.f));
    target.property.setFillColor(sf::Color::Red);

    if (GLOBAL::display_grid)
    {
        configureGrid(GLOBAL::cell_size, &this->grid);
    }
}

Scene::~Scene()
{
    delete instance;
}

Scene *Scene::getInstance()
{
    if (!instance)
        instance = new Scene();

    return instance;
}

void Scene::update(float dt)
{
    sf::Vector2f disp = target.property.getPosition() - joints[0].property.getPosition();
    float dist = Math::_length(disp);
    if (dist >= (upperarm_length + forearm_length))
    {
        disp = target.property.getPosition() - joints[1].property.getPosition();
        float angle = Math::_atan2(disp.y, disp.x);
        target.property.setPosition(joints[1].property.getPosition() + sf::Vector2f(Math::_cos(angle), Math::_sin(angle)) * forearm_length);
    }

    joints[2].property.move((target.property.getPosition() - joints[2].property.getPosition()) * 0.3f);

    // ik
    solveIK();
    // alignment
    alignLink();
}

void Scene::render(sf::RenderTarget *target)
{
    for (Circle &circle : this->joints)
        circle.render(target);
    for (Line &line : this->links)
        line.render(target);

    this->target.render(target);
}

void Scene::getMousePos(sf::Vector2f mouse_position)
{
    target.property.setPosition(mouse_position);
}

void Scene::alignLink()
{
    this->links.clear();

    for (int i = 0; i < this->joints.size() - 1; i++)
    {
        Line line = Line(this->joints[i].property.getPosition(), this->joints[i + 1].property.getPosition());
        this->links.push_back(line);
    }
}

void Scene::alignJoint(sf::Vector2f elbow_pos)
{
    joints[1].property.setPosition(elbow_pos);
}

void Scene::solveIK()
{
    float distance = Math::_length(joints[2].property.getPosition() - target.property.getPosition());
    if (distance < epsilon)
        return;

    this->shoulder_hand_length = Math::_length(joints[2].property.getPosition() - joints[0].property.getPosition());
    float theta = Math::_acos((pow(upperarm_length, 2) + pow(shoulder_hand_length, 2) - pow(forearm_length, 2)) / (2 * upperarm_length * shoulder_hand_length));
    float phi = Math::_atan2(joints[2].property.getPosition().y - joints[0].property.getPosition().y, joints[2].property.getPosition().x - joints[0].property.getPosition().x);

    sf::Vector2f elbow_position = joints[0].property.getPosition() + sf::Vector2f(Math::_cos(theta + phi), Math::_sin(theta + phi)) * upperarm_length;

    alignJoint(elbow_position);
}
