#pragma once
#include <algorithm>
#include <cmath>


namespace planner {
    template <typename Point>
    struct Heuristic {
        [[nodiscard]] virtual double operator ()(Point a, Point b) const = 0;
        virtual ~Heuristic() = default;
    };

    // todo: decouple dependency on 2 component vector with fields `x`, `y`
    template <typename Point>
    struct Diagonal : Heuristic<Point> {
        [[nodiscard]] double operator ()(Point a, Point b) const override {
            auto dx = std::max(a.x, b.x) - std::min(a.x, b.x);
            auto dy = std::max(a.y, b.y) - std::min(a.y, b.y);
            return static_cast<int>(std::max(dx, dy) - std::min(dx, dy)) + std::sqrt(2) * static_cast<int>(std::min(dx, dy));
        }
    };

    template <typename Point>
    struct Manhattan : Heuristic<Point> {
        [[nodiscard]] double operator ()(Point a, Point b) const override {
            auto dx = std::max(a.x, b.x) - std::min(a.x, b.x);
            auto dy = std::max(a.y, b.y) - std::min(a.y, b.y);
            return dx + dy;
        }
    };

    template <typename Point>
    struct Euclidean : Heuristic<Point> {
        [[nodiscard]] double operator ()(Point a, Point b) const override {
            auto dx = std::max(a.x, b.x) - std::min(a.x, b.x);
            auto dy = std::max(a.y, b.y) - std::min(a.y, b.y);
            return std::sqrt(dx * dx + dy * dy);
        }
    };


    template <typename Point>
    struct Chebyshev : Heuristic<Point> {
        [[nodiscard]] double operator ()(Point a, Point b) const override {
            auto dx = std::max(a.x, b.x) - std::min(a.x, b.x);
            auto dy = std::max(a.y, b.y) - std::min(a.y, b.y);
            return std::max(dx, dy);
        }
    };
}
