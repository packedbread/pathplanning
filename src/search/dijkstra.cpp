#include "dijkstra.hpp"


namespace planner {
    Dijkstra::Dijkstra(std::shared_ptr<Heuristic<Point>> heuristic, std::shared_ptr<TieBreaker<Point>> tie_breaker, const Options& options) :
        Search(std::move(heuristic), std::move(tie_breaker), options)
    {}

    void Dijkstra::search() const {

    }
}
