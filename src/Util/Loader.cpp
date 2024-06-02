#include "../../include/Util/Loader.hpp"

void Texture::load(sf::Texture *texture, const char *path)
{
    try
    {
        texture->loadFromFile(path);
    }
    catch (...)
    {
        std::cerr << "Could not load image" << std::endl;
    }
}

void Font::load(sf::Font *font, const char *path)
{
    try
    {
        font->loadFromFile(path);
    }
    catch (...)
    {
        std::cerr << "Could not load font" << std::endl;
    }
}