#include "float_compare.hpp"
#include "interface.hpp"
#include "tiebreaker.hpp"


namespace planner {
    bool finalize(Point a, Point b) {
        if (a.x == b.x) {
            return a.y < b.y;
        }
        return a.x < b.x;
    }

    bool GMax::is_better(const Node& a, const Node& b) const {
        if (very_close_equals(a.distance, b.distance)) {
            return finalize(a.position, b.position);
        }
        return a.distance > b.distance;
    }

    bool GMin::is_better(const Node& a, const Node& b) const {
        if (very_close_equals(a.estimation, b.estimation)) {
            return finalize(a.position, b.position);
        }
        return a.estimation > b.estimation;
    }
}
