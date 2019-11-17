#pragma once
#include <list>
#include <ostream>
#include <memory>
#include "heuristic.hpp"
#include "tiebreaker.hpp"
#include "../map.hpp"  // todo: figure out how to remove relative location dependency
#include "../quadratic.hpp"


namespace planner {
    struct Options {
        double heuristic_weight;
        bool allow_diagonal;
        bool cut_corners;
        bool allow_squeeze;

        bool operator == (const Options& options) const {
            return heuristic_weight == options.heuristic_weight &&
                allow_diagonal == options.allow_diagonal &&
                cut_corners == options.cut_corners &&
                allow_squeeze == options.allow_squeeze;
        }
    };

    std::ostream& operator << (std::ostream& out, const Options& options);

    struct Node {
        Point position;
        number distance;
        number estimation;

        Node(Point position, number distance, number estimation);

        [[nodiscard]] number value() const;
    };

    class SearchState {
        std::shared_ptr<Node> expanded_from;
        std::list<std::shared_ptr<Node>> path;
        std::list<std::shared_ptr<Node>> open;
        std::list<std::shared_ptr<Node>> closed;
    };

    // todo: refactor Search to accept heuristic, tie breaker and options as template parameters
    class Search {
    protected:
        std::shared_ptr<Heuristic<Point>> heuristic;
        std::shared_ptr<TieBreaker> tie_breaker;
        Options options;

        [[nodiscard]] bool is_better(const Node& a, const Node& b) const;
    public:
        Search(std::shared_ptr<Heuristic<Point>> heuristic, std::shared_ptr<TieBreaker> tie_breaker, const Options& options);

        [[nodiscard]] const std::shared_ptr<Heuristic<Point>>& get_heuristic() const;
        [[nodiscard]] const std::shared_ptr<TieBreaker>& get_tie_breaker() const;
        [[nodiscard]] const Options& get_options() const;

        [[nodiscard]] virtual SearchState search() const = 0;
    };
}
