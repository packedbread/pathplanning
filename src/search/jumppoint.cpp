#include "jumppoint.hpp"


namespace planner {
    JumpPoint::JumpPoint(std::shared_ptr<Heuristic<Point>> heuristic, std::shared_ptr<TieBreaker<Point>> tie_breaker, const Options& options) :
        Search(std::move(heuristic), std::move(tie_breaker), options)
    {}

    void JumpPoint::search() const {

    }
}
