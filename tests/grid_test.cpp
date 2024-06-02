#include "catch.hpp"
#include "../include/Util/Grid.hpp"
#include "../include/Entities/Box.hpp"

TEST_CASE("Grid Test")
{
    std::vector<std::vector<Box>> grid;
    const uint size = 10;
    SECTION("Configure Grid")
    {
        configureGrid(size, &grid);

        uint grid_row = GLOBAL::window_height / size;

        REQUIRE(grid.size() == grid_row);
    }
}