#include "../../include/Game/Scene.hpp"

Scene *Scene::instance = nullptr;

Scene::Scene()
{
    // setting up right leg joints
    Circle joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f - 50.f, GLOBAL::window_height / 2.f));
    this->right_joints.push_back(joint);
    joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f, GLOBAL::window_height / 2.f + 50.f));
    this->right_joints.push_back(joint);
    joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f + 50.f, GLOBAL::window_height / 2.f));
    this->right_joints.push_back(joint);

    this->upperarm_length = Math::_length(right_joints[1].property.getPosition() - right_joints[0].property.getPosition());
    this->forearm_length = Math::_length(right_joints[2].property.getPosition() - right_joints[1].property.getPosition());

    // setting up left leg joints
    joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f + 50.f, GLOBAL::window_height / 2.f));
    this->left_joints.push_back(joint);
    joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f + 100.f, GLOBAL::window_height / 2.f + 50.f));
    this->left_joints.push_back(joint);
    joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f + 150.f, GLOBAL::window_height / 2.f));
    this->left_joints.push_back(joint);

    // target...
    target = Circle(10.f, right_joints[2].property.getPosition() - sf::Vector2f(50.f, 100.f));
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
    outOfReach(right_joints, target);
    outOfReach(left_joints, target);
    // ik
    solveIK(right_joints);
    solveIK(left_joints);
    // alignment
    alignLink(right_links, right_joints);
    alignLink(left_links, left_joints);
}

void Scene::render(sf::RenderTarget *target)
{
    for (Circle &circle : this->right_joints)
        circle.render(target);
    for (Line &line : this->right_links)
        line.render(target);
    for (Circle &circle : this->left_joints)
        circle.render(target);
    for (Line &line : this->left_links)
        line.render(target);

    this->target.render(target);
}

void Scene::getMousePos(sf::Vector2f mouse_position)
{
    target.property.setPosition(mouse_position);
}

void Scene::alignLink(std::vector<Line> &links, std::vector<Circle> &joints)
{
    links.clear();

    for (int i = 0; i < this->right_joints.size() - 1; i++)
    {
        Line line = Line(joints[i].property.getPosition(), joints[i + 1].property.getPosition());
        links.push_back(line);
    }
}

void Scene::alignJoint(sf::Vector2f elbow_pos, std::vector<Circle> &joints)
{
    joints[1].property.setPosition(elbow_pos);
}

void Scene::solveIK(std::vector<Circle> &joints)
{
    float distance = Math::_length(joints[2].property.getPosition() - target.property.getPosition());
    if (distance < epsilon)
        return;

    this->shoulder_hand_length = Math::_length(joints[2].property.getPosition() - joints[0].property.getPosition());
    float theta = Math::_acos((pow(upperarm_length, 2) + pow(shoulder_hand_length, 2) - pow(forearm_length, 2)) / (2 * upperarm_length * shoulder_hand_length));
    float phi = Math::_atan2(joints[2].property.getPosition().y - joints[0].property.getPosition().y, joints[2].property.getPosition().x - joints[0].property.getPosition().x);

    sf::Vector2f elbow_position = joints[0].property.getPosition() + sf::Vector2f(Math::_cos(theta + phi), Math::_sin(theta + phi)) * upperarm_length;

    alignJoint(elbow_position, joints);
}

void Scene::outOfReach(std::vector<Circle> &joints, Circle &target_)
{
    sf::Vector2f disp = target_.property.getPosition() - joints[0].property.getPosition();
    float dist = Math::_length(disp);
    if (dist >= (upperarm_length + forearm_length))
    {
        // rotating the elbow towards the target
        disp = target_.property.getPosition() - joints[0].property.getPosition();
        float angle = Math::_atan2(disp.y, disp.x);
        joints[1].property.setPosition(joints[0].property.getPosition() + sf::Vector2f(Math::_cos(angle), Math::_sin(angle)) * upperarm_length);

        // target always on range for the elbow to be aligned
        // and the hand to always follow the target, whether it's out of reach or not
        disp = target_.property.getPosition() - joints[1].property.getPosition();
        angle = Math::_atan2(disp.y, disp.x);
        target_.property.setPosition(joints[1].property.getPosition() + sf::Vector2f(Math::_cos(angle), Math::_sin(angle)) * forearm_length);
    }

    joints[2].property.move((target_.property.getPosition() - joints[2].property.getPosition()) * 0.3f);
}
