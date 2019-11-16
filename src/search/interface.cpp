#include "interface.hpp"


namespace planner {
    Search::Search(std::shared_ptr<Heuristic<Point>> heuristic, std::shared_ptr<TieBreaker<Point>> tie_breaker, const Options& options) :
        heuristic(std::move(heuristic)),
        tie_breaker(std::move(tie_breaker)),
        options(options)
    {}

    const std::shared_ptr<Heuristic<Point>>& Search::get_heuristic() const {
        return heuristic;
    }

    const std::shared_ptr<TieBreaker<Point>>& Search::get_tie_breaker() const {
        return tie_breaker;
    }

    const Options& Search::get_options() const {
        return options;
    }

    std::ostream& operator<<(std::ostream& out, const Options& options) {
        return out << "Options{ " <<
                   options.heuristic_weight << ", " <<
                   options.allow_diagonal << ", " <<
                   options.cut_corners << ", " <<
                   options.allow_squeeze << " }";
    }
}
