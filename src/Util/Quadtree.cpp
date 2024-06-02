#include "../../include/Util/Quadtree.hpp"

Quad::Quad()
{
    topLeft = sf::Vector2f(0, 0);
    botRight = sf::Vector2f(0, 0);

    node = nullptr;
    ne = nullptr;
    nw = nullptr;
    sw = nullptr;
    se = nullptr;
}

Quad::Quad(sf::Vector2f topLeft_, sf::Vector2f botRight_)
{
    topLeft = topLeft_;
    botRight = botRight_;
    sf::Vector2f size = sf::Vector2f(botRight.x - topLeft.x, botRight.y - topLeft.y);
    sf::Vector2f position = sf::Vector2f((topLeft.x + botRight.x) / 2.f, (topLeft.y + botRight.y) / 2.f);

    box = Box(size, position);

    node = nullptr;
    ne = nullptr;
    nw = nullptr;
    sw = nullptr;
    se = nullptr;
}

void Quad::insert(Circle *n)
{
    if (!collision._boxPointCollide(this->box, n->property.getPosition()))
        return;

    if (this->node == nullptr && !subdivided)
    {
        this->node = n;
        subdivided = true;
        return;
    }

    sf::Vector2f mid = sf::Vector2f((topLeft.x + botRight.x) / 2, (topLeft.y + botRight.y) / 2);

    if (n->property.getPosition().x < (topLeft.x + botRight.x) / 2)
    {
        if (n->property.getPosition().y < (topLeft.y + botRight.y) / 2)
        {
            nw = new Quad(topLeft, mid);
            nw->insert(n); // North-West
        }
        else
        {
            sw = new Quad(sf::Vector2f(topLeft.x, mid.y), sf::Vector2f(mid.x, botRight.y));
            sw->insert(n); // South-West
        }
    }
    else
    {
        if (n->property.getPosition().y < (topLeft.y + botRight.y) / 2)
        {
            ne = new Quad(sf::Vector2f(mid.x, topLeft.y), sf::Vector2f(botRight.x, mid.y));
            ne->insert(n); // North-East
        }
        else
        {
            se = new Quad(mid, botRight);
            se->insert(n); // South-East
        }
    }
}

std::vector<Circle *> Quad::search(sf::Vector2f topLeft_, sf::Vector2f botRight_)
{
    std::vector<Circle *> found;
    sf::Vector2f size = sf::Vector2f(botRight_.x - topLeft_.x, botRight_.y - topLeft_.y);
    sf::Vector2f position = sf::Vector2f((topLeft_.x + botRight_.x) / 2.f, (topLeft_.y + botRight_.y) / 2.f);
    Box range = Box(size, position);

    if (!collision._boxCollide(this->box, range))
    {
        return found;
    }

    if (this->node != nullptr)
    {
        if (collision._boxPointCollide(range, this->node->property.getPosition()))
        {
            found.push_back(this->node);
        }
    }

    if (!subdivided)
    {
        return found;
    }

    if (nw != nullptr)
    {
        std::vector<Circle *> nw_found = nw->search(topLeft_, botRight_);
        found.insert(found.end(), nw_found.begin(), nw_found.end());
    }
    if (sw != nullptr)
    {
        std::vector<Circle *> sw_found = sw->search(topLeft_, botRight_);
        found.insert(found.end(), sw_found.begin(), sw_found.end());
    }
    if (ne != nullptr)
    {
        std::vector<Circle *> ne_found = ne->search(topLeft_, botRight_);
        found.insert(found.end(), ne_found.begin(), ne_found.end());
    }
    if (se != nullptr)
    {
        std::vector<Circle *> se_found = se->search(topLeft_, botRight_);
        found.insert(found.end(), se_found.begin(), se_found.end());
    }

    return found;
}

void Quad::clear()
{
    // Clear the nodes in the current quadrant
    node = nullptr;

    if (nw == nullptr)
        return;

    // Recursively clear nodes in child quadrants
    if (nw != nullptr)
    {
        nw->clear();
        delete nw;
        nw = nullptr;
    }
    if (sw != nullptr)
    {
        sw->clear();
        delete sw;
        sw = nullptr;
    }
    if (ne != nullptr)
    {
        ne->clear();
        delete ne;
        ne = nullptr;
    }
    if (se != nullptr)
    {
        se->clear();
        delete se;
        se = nullptr;
    }
}