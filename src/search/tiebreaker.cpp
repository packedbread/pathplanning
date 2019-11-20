#include "tiebreaker.hpp"
#include "interface.hpp"


namespace planner {
    bool finalize(Point a, Point b) {
        if (a.x == b.x) {
            return a.y < b.y;
        }
        return a.x < b.x;
    }

    bool GMax::is_better(const Node& a, const Node& b) const {
        if (a.distance == b.distance) {
            return finalize(a.position, b.position);
        }
        return evaluate(a.distance) > evaluate(b.distance);
    }

    bool GMin::is_better(const Node& a, const Node& b) const {
        if (a.estimation == b.estimation) {
            return finalize(a.position, b.position);
        }
        return evaluate(a.estimation) > evaluate(b.estimation);
    }
}
