#include "../../include/Game/Scene.hpp"

Scene *Scene::instance = nullptr;

Scene::Scene()
{
    // body
    body = Circle(50.f, sf::Vector2f(GLOBAL::window_width * 0.1f, GLOBAL::window_height - (upperarm_length + forearm_length)));
    body.property.setFillColor(sf::Color(171, 120, 78, 255));

    // target...
    right_target = Circle(10.f, body.property.getPosition() + sf::Vector2f(-body.property.getRadius() * 0.7f, (upperarm_length + forearm_length - 10.f)));
    right_target.property.setFillColor(sf::Color::Red);
    left_target = Circle(10.f, body.property.getPosition() + sf::Vector2f(body.property.getRadius() * 0.7f, (upperarm_length + forearm_length - 10.f)));
    left_target.property.setFillColor(sf::Color::Blue);

    // setting up right leg joints
    Circle joint = Circle(10.f, body.property.getPosition());
    this->right_joints.push_back(joint);
    joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f, GLOBAL::window_height / 2.f + 50.f));
    this->right_joints.push_back(joint);
    joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f + 50.f, GLOBAL::window_height / 2.f));
    this->right_joints.push_back(joint);

    // setting up left leg joints
    joint = Circle(10.f, body.property.getPosition());
    this->left_joints.push_back(joint);
    joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f + 100.f, GLOBAL::window_height / 2.f + 50.f));
    this->left_joints.push_back(joint);
    joint = Circle(10.f, sf::Vector2f(GLOBAL::window_width / 2.f + 150.f, GLOBAL::window_height / 2.f));
    this->left_joints.push_back(joint);

    // walk related
    l_start_pos = l_end_pos = l_mid_pos = left_target.property.getPosition();
    r_start_pos = r_end_pos = r_mid_pos = right_target.property.getPosition();
    step_speed = min_step_speed;
    body_height = body.property.getPosition().y;
    grounded_foot_height = right_target.property.getPosition().y;
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
    outOfReach(right_joints, right_target);
    outOfReach(left_joints, left_target);
    // ik
    solveIK(right_joints, right_target);
    solveIK(left_joints, left_target);
    // alignment
    alignHip();
    alignLink(right_links, right_joints);
    alignLink(left_links, left_joints);

    body_speed = Math::_map(step_speed_lerp, 1.f, 2.f);
    body_speed = Math::_clampOnRange(body_speed, 1.f, 2.f);

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
        body.property.move(sf::Vector2f(-body_speed, 0.f));
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
        body.property.move(sf::Vector2f(body_speed, 0.f));
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift))
    {
        if (step_speed_lerp < 1.f)
            step_speed_lerp += 1.f * dt;
    }
    if (!sf::Keyboard::isKeyPressed(sf::Keyboard::LShift))
        if (step_speed_lerp > 0.f)
            step_speed_lerp -= 1.f * dt;
    // walk
    solveWalk();

    right_target.property.setPosition(Math::_lerp(Math::_lerp(r_start_pos, r_mid_pos, right_lerp), Math::_lerp(r_mid_pos, r_end_pos, right_lerp), right_lerp));
    left_target.property.setPosition(Math::_lerp(Math::_lerp(l_start_pos, l_mid_pos, left_lerp), Math::_lerp(l_mid_pos, l_end_pos, left_lerp), left_lerp));
    step_speed = Math::_lerp(min_step_speed, max_step_speed, step_speed_lerp);

        // body bobbing
        if (right_lerp < 1.f)
    {
        float height = Math::_map(right_target.property.getPosition().y, grounded_foot_height, body_height);
        sf::Vector2f height_vector = sf::Vector2f(body.property.getPosition().x, height);
        sf::Vector2f tempo = height_vector - body.property.getPosition();
        body.property.move(tempo * 0.05f);
    }
    else if (left_lerp < 1.f)
    {
        float height = Math::_map(left_target.property.getPosition().y, grounded_foot_height, body_height);
        sf::Vector2f height_vector = sf::Vector2f(body.property.getPosition().x, height);
        sf::Vector2f tempo = height_vector - body.property.getPosition();
        body.property.move(tempo * 0.05f);
    }
    else
    {
        body.property.setPosition(body.property.getPosition().x, body_height);
    }

    right_lerp += step_speed * dt;
    left_lerp += step_speed * dt;
    step_size = 0.f;
}

void Scene::render(sf::RenderTarget *target)
{
    for (Circle &circle : this->left_joints)
        circle.render(target);
    for (Line &line : this->left_links)
        line.render(target);

    this->body.render(target);

    for (Circle &circle : this->right_joints)
        circle.render(target);
    for (Line &line : this->right_links)
        line.render(target);

    // this->left_target.render(target);
    // this->right_target.render(target);

    // gizmo.drawRay(body.property.getPosition(), body.property.getPosition() + sf::Vector2f(0.f, (upperarm_length + forearm_length)));
    // gizmo.drawCircle(body.property.getPosition() + sf::Vector2f(0.f, (upperarm_length + forearm_length)), 10.f, sf::Color::Green);
    // gizmo.drawAll(target);
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

void Scene::solveIK(std::vector<Circle> &joints, Circle &target)
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
    sf::Vector2f fake_target = target_.property.getPosition();
    sf::Vector2f disp = target_.property.getPosition() - joints[0].property.getPosition();
    float dist = Math::_length(disp);
    if (dist >= (upperarm_length + forearm_length))
    {
        // positioning the elbow towards the target
        disp = target_.property.getPosition() - joints[0].property.getPosition();
        float angle = Math::_atan2(disp.y, disp.x);
        joints[1].property.setPosition(joints[0].property.getPosition() + sf::Vector2f(Math::_cos(angle), Math::_sin(angle)) * upperarm_length);

        // target always on range for the elbow to be aligned
        // and the hand to always follow the target, whether it's out of reach or not
        disp = target_.property.getPosition() - joints[1].property.getPosition();
        angle = Math::_atan2(disp.y, disp.x);
        fake_target = sf::Vector2f(joints[1].property.getPosition() + sf::Vector2f(Math::_cos(angle), Math::_sin(angle)) * forearm_length);
    }

    joints[2].property.move((fake_target - joints[2].property.getPosition()) * 0.3f);
}

void Scene::alignHip()
{
    right_joints[0].property.setPosition(body.property.getPosition());
    left_joints[0].property.setPosition(body.property.getPosition());
}

bool Scene::inBalance(float x, float y, float value)
{
    float min = Math::_min(x, y);
    float max = Math::_max(x, y);

    return value >= min && value <= max;
}

void Scene::newStep(sf::Vector2f &start_pos, sf::Vector2f &end_pos, sf::Vector2f &mid_pos, Circle &foot_target, float &current_foot_lerp)
{
    start_pos = end_pos;

    current_foot_lerp = 0.f;

    // if right foot
    if (foot_target.property.getFillColor() == sf::Color::Red)
        end_pos = (body.property.getPosition() + sf::Vector2f(0.f, (upperarm_length + forearm_length - 10.f))) - sf::Vector2f(foot_distance_on_x, 0.f);
    // if left foot
    else
        end_pos = (body.property.getPosition() + sf::Vector2f(0.f, (upperarm_length + forearm_length - 10.f))) + sf::Vector2f(foot_distance_on_x, 0.f);

    end_pos += sf::Vector2f(over_shoot_factor, 0.f);

    // for lifting
    step_size = Math::_length(end_pos - start_pos);
    mid_pos = (start_pos + end_pos) / 2.f;
    mid_pos.y -= step_size;
}

void Scene::solveWalk()
{
    if (!inBalance(right_joints[2].property.getPosition().x, left_joints[2].property.getPosition().x, body.property.getPosition().x))
    {
        // the other foot should finish its step before the current foot proceeds with its own
        if (left_lerp > 4.f && right_lerp > left_lerp)
            newStep(r_start_pos, r_end_pos, r_mid_pos, right_target, right_lerp);
        if (right_lerp > 4.f && left_lerp > right_lerp)
            newStep(l_start_pos, l_end_pos, l_mid_pos, left_target, left_lerp);
    }
}