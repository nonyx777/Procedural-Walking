#pragma once

#include "../GameObject.hpp"
#include "../Entities/Entities.hpp"
#include "../Util/Gizmo.hpp"
#include "../Util/Grid.hpp"
#include "../Globals.hpp"
#include "../Util/Collision.hpp"
#include <glm/glm.hpp>

class Scene : public GameObject
{
private:
    static Scene *instance;
    Gizmo gizmo;

    std::vector<Circle> right_joints;
    std::vector<Circle> left_joints;
    std::vector<Line> right_links;
    std::vector<Line> left_links;

    Circle body;

    Circle right_target;
    Circle left_target;

    // IK related
    float upperarm_length = 70.f;
    float forearm_length = 70.f;
    float shoulder_hand_length;
    float epsilon = 0.1f;

    // walk related
    float lerp = 0.f;
    sf::Vector2f r_start_pos, r_end_pos, r_mid_pos;
    sf::Vector2f l_start_pos, l_end_pos, l_mid_pos;
    float over_shoot_factor;
    float step_speed = 8.f;
    float foot_distance_on_x;

private:
    Scene();
    ~Scene();

public:
    // Delete copy constructor and assignment operator to prevent cloning
    Scene(const Scene &) = delete;
    Scene &operator=(const Scene &) = delete;

    static Scene *getInstance();

    void update(float dt) override;
    void update(sf::Vector2f &vec, float dt);
    void render(sf::RenderTarget *target) override;

    // joint and link
    void alignLink(std::vector<Line> &links, std::vector<Circle> &joints);
    void alignJoint(sf::Vector2f elbow_pos, std::vector<Circle> &joints);
    void alignHip();

    // IK related
    void solveIK(std::vector<Circle> &joints, Circle &target);
    void outOfReach(std::vector<Circle> &joints, Circle &target_);

    // walk related
    void solveWalk();
    bool inBalance(float x, float y, float value);
    void newStep(sf::Vector2f &start_pos, sf::Vector2f &end_pos, Circle &foot_target);
};