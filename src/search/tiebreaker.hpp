#pragma once


namespace planner {
    struct Node;

    /// TieBreaker interface
    /// invocation contract: only if node values are the same
    /// operator (): returns `true` if `a` is better than `b`
    struct TieBreaker {
        [[nodiscard]] virtual bool operator ()(const Node& a, const Node& b) const = 0;
        virtual ~TieBreaker() = default;
    };

    struct GMax : TieBreaker {
        [[nodiscard]] bool operator ()(const Node &a, const Node &b) const override;
    };

    struct GMin : TieBreaker {
        [[nodiscard]] bool operator ()(const Node &a, const Node &b) const override;
    };
}
