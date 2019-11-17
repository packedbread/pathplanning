#include "interface.hpp"


namespace planner {
    Node::Node(Point position, number distance, number estimation)  :
        position(position),
        distance(distance),
        estimation(estimation)
    {}

    number Node::value() const  {
        return distance + estimation;
    }

    Search::Search(std::shared_ptr<Heuristic<Point>> heuristic, std::shared_ptr<TieBreaker> tie_breaker, const Options& options) :
        heuristic(std::move(heuristic)),
        tie_breaker(std::move(tie_breaker)),
        options(options)
    {}

    const std::shared_ptr<Heuristic<Point>>& Search::get_heuristic() const {
        return heuristic;
    }

    const std::shared_ptr<TieBreaker>& Search::get_tie_breaker() const {
        return tie_breaker;
    }

    const Options& Search::get_options() const {
        return options;
    }

    bool Search::is_better(const Node& a, const Node& b) const {
        if (a.value() == b.value()) {
            if (tie_breaker->is_same(a, b)) {
                return FinalizingTieBreaker::is_better(a, b);
            }
            return tie_breaker->is_better(a, b);
        }
        return evaluate(a.value()) < evaluate(b.value());
    }

    std::ostream& operator<<(std::ostream& out, const Options& options) {
        return out << "Options{ " <<
                   options.heuristic_weight << ", " <<
                   options.allow_diagonal << ", " <<
                   options.cut_corners << ", " <<
                   options.allow_squeeze << " }";
    }
}
