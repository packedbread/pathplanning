#pragma once


namespace planner {
    struct Node;

    // invocation contract: only if node values are the same
    struct TieBreaker {
        [[nodiscard]] virtual bool is_better(const Node& a, const Node& b) const = 0;
        virtual ~TieBreaker() = default;
    };

    struct GMax : TieBreaker {
        [[nodiscard]] bool is_better(const Node &a, const Node &b) const override;
    };

    struct GMin : TieBreaker {
        [[nodiscard]] bool is_better(const Node &a, const Node &b) const override;
    };
}
