#include "../include/Window.hpp"

void Window::initVariables()
{
    this->window = nullptr;
}
void Window::initWindow()
{
    this->video_mode.width = 100;
    this->video_mode.height = 100;
    this->window = new sf::RenderWindow(this->video_mode, "Configuration Window", sf::Style::Titlebar | sf::Style::Close);
    this->window->setFramerateLimit(60);
}

Window::Window()
{
    this->initVariables();
}
Window::~Window()
{
    ImGui::SFML::Shutdown();
    delete this->window;
}

const bool Window::running() const
{
    return this->window->isOpen();
}

void Window::pollEvent()
{
    while (this->window->pollEvent(this->event))
    {
        this->eventImGui();
        switch (this->event.type)
        {
        case sf::Event::Closed:
            this->window->close();
            break;
        }
    }
}

void Window::update(float dt)
{
    this->pollEvent();

    this->updateImGui();
}

void Window::render(sf::RenderTarget *target)
{
    //....
}
void Window::winRender()
{
    this->window->clear(sf::Color::Black);
    this->renderImGui();
    this->window->display();
}

void Window::initImGui()
{
    ImGui::SFML::Init(*window, true);
}
void Window::eventImGui()
{
    ImGui::SFML::ProcessEvent(this->event);
}
void Window::updateImGui()
{
    ImGui::SFML::Update(*window, this->deltaClock.restart());
}
void Window::renderImGui()
{
    ImGui::SFML::Render(*window);
}