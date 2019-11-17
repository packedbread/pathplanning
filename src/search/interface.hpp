#pragma once
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

    struct Node {};  // todo: implement node interface

    class Search {
        std::shared_ptr<Heuristic<Point>> heuristic;
        std::shared_ptr<TieBreaker<Point>> tie_breaker;
        Options options;
    public:
        Search(std::shared_ptr<Heuristic<Point>> heuristic, std::shared_ptr<TieBreaker<Point>> tie_breaker, const Options& options);

        [[nodiscard]] const std::shared_ptr<Heuristic<Point>>& get_heuristic() const;
        [[nodiscard]] const std::shared_ptr<TieBreaker<Point>>& get_tie_breaker() const;
        [[nodiscard]] const Options& get_options() const;

        virtual void search() const = 0;
    };
}
