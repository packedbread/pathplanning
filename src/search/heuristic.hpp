#pragma once
#include <algorithm>
#include <cmath>
#include "../quadratic.hpp"


namespace planner {
    template <typename Point>
    struct Heuristic {
        virtual number operator ()(Point a, Point b) const = 0;
    };

    // todo: decouple dependency on 2 component vector with fields `x`, `y`
    template <typename Point>
    struct Diagonal : Heuristic<Point> {
        number operator ()(Point a, Point b) const override {
            auto dx = std::max(a.x, b.x) - std::min(a.x, b.x);
            auto dy = std::max(a.y, b.y) - std::min(a.y, b.y);
            return { static_cast<int>(std::max(dx, dy) - std::min(dx, dy)), static_cast<int>(std::min(dx, dy)) };
        }
    };

    template <typename Point>
    struct Manhattan : Heuristic<Point> {
        number operator ()(Point a, Point b) const override {
            auto dx = std::max(a.x, b.x) - std::min(a.x, b.x);
            auto dy = std::max(a.y, b.y) - std::min(a.y, b.y);
            return static_cast<number>(dx + dy);
        }
    };

    template <typename Point>
    struct Euclidean : Heuristic<Point> {
        number operator ()(Point a, Point b) const override {
            auto dx = std::max(a.x, b.x) - std::min(a.x, b.x);
            auto dy = std::max(a.y, b.y) - std::min(a.y, b.y);
            return std::sqrt(dx * dx + dy * dy);
        }
    };


    template <typename Point>
    struct Chebyshev : Heuristic<Point> {
        number operator ()(Point a, Point b) const override {
            auto dx = std::max(a.x, b.x) - std::min(a.x, b.x);
            auto dy = std::max(a.y, b.y) - std::min(a.y, b.y);
            return static_cast<number>(std::max(dx, dy));
        }
    };
}
