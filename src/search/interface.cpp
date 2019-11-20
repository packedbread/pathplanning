#include "interface.hpp"


namespace planner {
    std::ostream& operator << (std::ostream& out, const Options& options) {
        return out << "Options{ " <<
                   options.heuristic_weight << ", " <<
                   options.allow_diagonal << ", " <<
                   options.cut_corners << ", " <<
                   options.allow_squeeze << " }";
    }

    Node::Node(Point position, number distance, number estimation, std::shared_ptr<Node> expanded_from)  :
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
        if (path.empty()) {
            return 0.0;
        }
        number result;
        Manhattan<Point> metric;
        for (auto it = std::next(std::begin(path)); it != std::end(path); ++it) {
            result += metric((*it)->position, (*it)->expanded_from->position).real < 2 ? 1 : number{ 0, 1 };
        }
        return evaluate(result);
    }

    Search::Comparator::Comparator(const Search& search) : search(search) {}

    bool Search::Comparator::operator () (const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
        // order of operands is changed intentionally, due to priority queue nature
        // of returning elements with highest priority first, but using a strict weak ordering
        if (a->distance + a->estimation == b->distance + b->estimation) {
            return search.tie_breaker->is_better(*b, *a);
        }
        return evaluate(b->distance) + search.options.heuristic_weight * evaluate(b->estimation) <
            evaluate(a->distance) + search.options.heuristic_weight * evaluate(a->estimation);
    }
}
