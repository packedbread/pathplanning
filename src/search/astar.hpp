#pragma once
#include "interface.hpp"


namespace planner {
    class AStar : public Search {
    public:
        AStar(std::shared_ptr<Heuristic<Point>> heuristic, std::shared_ptr<TieBreaker> tie_breaker, const Options& options);

        [[nodiscard]] SearchState search() const override;
    };
}
