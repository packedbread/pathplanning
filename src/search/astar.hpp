#pragma once
#include "interface.hpp"


namespace planner {
    class AStar : public Search {
    public:
        AStar(std::shared_ptr<Heuristic<Point>> heuristic, std::shared_ptr<TieBreaker<Point>> tie_breaker, const Options& options);

        void search() const override;
    };
}
