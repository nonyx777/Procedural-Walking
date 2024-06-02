#pragma once
#include <iostream>
#include <SFML/Graphics.hpp>
#include "GameObject.hpp"
#include "../library/UI/imgui.h"
#include "../library/UI/imgui-SFML.h"
#include "Globals.hpp"

using namespace std;

class Window : public GameObject
{
protected:
    // initializer functions
    void initVariables();
    virtual void initWindow();

public:
    // window variables
    sf::RenderWindow *window;

protected:
    sf::VideoMode video_mode;
    sf::Event event;

    // mouse position
    sf::Vector2i mouse_position;
    sf::Vector2f mouse_position_view;

    // ui related
    sf::Clock deltaClock;
    sf::Time deltaTime;

public:
    Window();
    ~Window();

    virtual void pollEvent();
    void update(float dt) override;
    void render(sf::RenderTarget *target) override;
    void winRender();

    const bool running() const;

    // ui related
    void initImGui();
    void eventImGui();
    void updateImGui();
    void renderImGui();
};
