#pragma once
#include "interface.hpp"


namespace planner {
    class Dijkstra : public Search {
    public:
        Dijkstra(std::shared_ptr<Heuristic<Point>> heuristic, std::shared_ptr<TieBreaker> tie_breaker, const Options& options);

        [[nodiscard]] SearchState search() const override;
    };
}
