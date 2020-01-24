#include "float_compare.hpp"
#include "interface.hpp"


namespace planner {
    std::ostream& operator << (std::ostream& out, const Options& options) {
        return out << "Options{ " <<
                   options.heuristic_weight << ", " <<
                   options.allow_diagonal << ", " <<
                   options.cut_corners << ", " <<
                   options.allow_squeeze << " }";
    }

    Node::Node(Point position, double distance, double estimation, std::shared_ptr<Node> expanded_from)  :
        position(position),
        distance(distance),
        estimation(estimation),
        expanded_from(std::move(expanded_from))
    {}

    std::ostream& operator << (std::ostream& out, const Node& node) {
        out << "Node{ " << node.position << ", " << node.distance << ", " << node.estimation;
        if (node.expanded_from != nullptr) {
            out << ", " << node.expanded_from->position;
        }
        return out << " }";
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

    double SearchState::path_length() const {
        if (path.empty()) {  // todo: is this necessary?
            return 0.0;
        }
        double result = 0.0;
        Euclidean<Point> metric;
        for (auto it = std::next(std::begin(path)); it != std::end(path); ++it) {
            result += metric((*it)->position, (*it)->expanded_from->position);
        }
        return result;
    }

    Search::Comparator::Comparator(const Search& search) : search(search) {}

    bool Search::Comparator::operator () (const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
        // order of operands is changed intentionally, due to priority queue nature
        // of returning elements with highest priority first, but using a strict weak ordering
        if (very_close_equals(a->distance + search.options.heuristic_weight * a->estimation * (1.0 - 1e-3), b->distance + search.options.heuristic_weight * b->estimation * (1.0 - 1e-3))) {
            return search.tie_breaker->is_better(*b, *a);
        }
        return a->distance + search.options.heuristic_weight * a->estimation * (1.0 - 1e-3) > b->distance + search.options.heuristic_weight * b->estimation * (1.0 - 1e-3);
    }
}
