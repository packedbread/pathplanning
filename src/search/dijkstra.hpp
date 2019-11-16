#pragma once
#include "interface.hpp"


namespace planner {
    class Dijkstra : public Search {
    public:
        Dijkstra(std::shared_ptr<Heuristic<Point>> heuristic, std::shared_ptr<TieBreaker<Point>> tie_breaker, const Options& options);

        void search() const override;
    };
}
