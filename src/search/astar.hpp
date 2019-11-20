#pragma once
#include "interface.hpp"


namespace planner {
    class AStar : public Search {
    public:
        using Search::Search;

        [[nodiscard]] SearchState search(Point from, Point to, const GridMap<CellType>& map) const override;
    };
}
