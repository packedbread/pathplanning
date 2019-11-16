#pragma once
#include <algorithm>
#include <cmath>


namespace planner {
    template <typename Point>
    struct Heuristic {
        virtual double operator ()(Point a, Point b) const = 0;
    };

    // todo: decouple dependency on 2 component vector with fields `x`, `y`
    template <typename Point>
    struct Diagonal : Heuristic<Point> {
        double operator ()(Point a, Point b) const override {
            auto dx = std::max(a.x, b.x) - std::min(a.x, b.x);
            auto dy = std::max(a.y, b.y) - std::min(a.y, b.y);
            return 0; // todo: implement diagonal heuristic
        }
    };

    template <typename Point>
    struct Manhattan : Heuristic<Point> {
        double operator ()(Point a, Point b) const override {
            auto dx = std::max(a.x, b.x) - std::min(a.x, b.x);
            auto dy = std::max(a.y, b.y) - std::min(a.y, b.y);
            return static_cast<double>(dx + dy);
        }
    };

    template <typename Point>
    struct Euclidean : Heuristic<Point> {
        double operator ()(Point a, Point b) const override {
            auto dx = std::max(a.x, b.x) - std::min(a.x, b.x);
            auto dy = std::max(a.y, b.y) - std::min(a.y, b.y);
            return std::sqrt(dx * dx + dy * dy);
        }
    };


    template <typename Point>
    struct Chebyshev : Heuristic<Point> {
        double operator ()(Point a, Point b) const override {
            auto dx = std::max(a.x, b.x) - std::min(a.x, b.x);
            auto dy = std::max(a.y, b.y) - std::min(a.y, b.y);
            return static_cast<double>(std::max(dx, dy));
        }
    };
}
