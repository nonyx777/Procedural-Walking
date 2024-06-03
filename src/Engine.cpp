#include "../include/Engine.hpp"

Engine::Engine() : Window()
{
    this->initWindow();
    this->initImGui();
}

void Engine::initWindow()
{
    this->video_mode.width = GLOBAL::window_width;
    this->video_mode.height = GLOBAL::window_height;
    this->window = new sf::RenderWindow(this->video_mode, "Proto", sf::Style::Titlebar | sf::Style::Resize | sf::Style::Close);
    this->window->setFramerateLimit(60);
}

void Engine::update(float dt)
{
    this->pollEvent();
    this->mouse_position = sf::Mouse::getPosition(*this->window);
    this->mouse_position_view = this->window->mapPixelToCoords(this->mouse_position);

    scene->update(dt);
    this->updateImGui();
    ImGui::Begin("Proto");
    ImGui::Text("FPS: %f", GLOBAL::fps);
    ImGui::End();
}
void Engine::render(sf::RenderTarget *target)
{
    //....
}
void Engine::render()
{
    this->window->clear(sf::Color::Black);
    scene->render(this->window);
    this->renderImGui();
    this->window->display();
}