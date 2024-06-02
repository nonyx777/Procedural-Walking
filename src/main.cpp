#include "../include/Engine.hpp"
#include "../include/ConfigWindow.hpp"
#include "../library/UI/imgui.h"
#include "../library/UI/imgui-SFML.h"

int main()
{
    sf::Clock clock;

    // Create a second SFML window for ImGui rendering
    ConfigWindow configWindow;
    while (configWindow.running())
    {
        configWindow.update(0.f);
        configWindow.winRender();
    }

    Engine engine;
    while (engine.running())
    {
        float dt = clock.restart().asSeconds();
        engine.update(dt);
        engine.render();
        GLOBAL::fps = 1.f/dt;
    }

    return 0;
}
