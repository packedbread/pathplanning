#include "thetastar.hpp"


namespace planner {
    ThetaStar::ThetaStar(std::shared_ptr<Heuristic<Point>> heuristic, std::shared_ptr<TieBreaker> tie_breaker, const Options& options) :
        Search(std::move(heuristic), std::move(tie_breaker), options)
    {}

    SearchState ThetaStar::search() const {

    }
}
