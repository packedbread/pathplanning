#pragma once
#include <chrono>
#include <list>
#include <ostream>
#include <memory>
#include "heuristic.hpp"
#include "tiebreaker.hpp"
#include "../map.hpp"  // todo: figure out how to remove relative location dependency


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
        double distance;
        double estimation;
        std::shared_ptr<Node> expanded_from;

        Node(Point position, double distance, double estimation, std::shared_ptr<Node> expanded_from = nullptr);
    };

    std::ostream& operator << (std::ostream& out, const Node& node);

    struct SearchState {
        bool path_found;
        std::list<std::shared_ptr<Node>> path;
        std::list<std::shared_ptr<Node>> open;
        std::list<std::shared_ptr<Node>> closed;
        std::chrono::high_resolution_clock::duration time_spent;

        [[nodiscard]] double path_length() const;
    };

    // todo: refactor Search to accept heuristic, tie breaker and options as template parameters
    // it may be hard or even impossible, because necessary types will be known only when the file is read
    // which will happen at runtime

    class Search {
    protected:
        std::shared_ptr<Heuristic<Point>> heuristic;
        std::shared_ptr<TieBreaker> tie_breaker;
        Options options;
    public:
        Search(std::shared_ptr<Heuristic<Point>> heuristic, std::shared_ptr<TieBreaker> tie_breaker, const Options& options);

        [[nodiscard]] const std::shared_ptr<Heuristic<Point>>& get_heuristic() const;
        [[nodiscard]] const std::shared_ptr<TieBreaker>& get_tie_breaker() const;
        [[nodiscard]] const Options& get_options() const;

        [[nodiscard]] virtual SearchState search(Point from, Point to, const GridMap<CellType>& map) const = 0;

        virtual ~Search() = default;
    };
}
