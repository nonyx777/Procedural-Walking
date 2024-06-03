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
    std::vector<std::vector<Box>> grid;

    std::vector<Circle> right_joints;
    std::vector<Circle> left_joints;
    std::vector<Line> right_links;
    std::vector<Line> left_links;

    Circle target;

    // IK related
    float upperarm_length, forearm_length, shoulder_hand_length;
    float epsilon = 0.1f;

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

    // get mouse position
    void getMousePos(sf::Vector2f mouse_position);

    // joint and link
    void alignLink(std::vector<Line> &links, std::vector<Circle> &joints);
    void alignJoint(sf::Vector2f elbow_pos, std::vector<Circle> &joints);

    // IK related
    void solveIK(std::vector<Circle> &joints);
    void outOfReach(std::vector<Circle> &joints, Circle &target_);
};