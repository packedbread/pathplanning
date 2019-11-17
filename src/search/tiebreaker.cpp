#include "tiebreaker.hpp"
#include "interface.hpp"


namespace planner {

    bool TieBreaker::is_same(const Node& a, const Node& b) const {
        return !is_better(a, b) && !is_better(b, a);
    }

    bool GMax::is_better(const Node& a, const Node& b) const {
        return evaluate(a.distance) > evaluate(b.distance);
    }

    bool GMin::is_better(const Node& a, const Node& b) const {
        return evaluate(a.estimation) > evaluate(b.estimation);
    }

    bool FinalizingTieBreaker::is_better(const Node& a, const Node& b) {
        if (a.position.x == b.position.x) {
            return a.position.y < b.position.y;
        }
        return a.position.x < b.position.x;
    }
}
