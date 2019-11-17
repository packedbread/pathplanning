#pragma once


namespace planner {
    class Node;

    struct TieBreaker {
        virtual bool is_better(const Node& a, const Node& b) const = 0;
        virtual bool is_same(const Node& a, const Node& b) const;
    };

    struct GMax : TieBreaker {
        bool is_better(const Node &a, const Node &b) const override;
    };

    struct GMin : TieBreaker {
        bool is_better(const Node &a, const Node &b) const override;
    };

    // todo: consider moving this into search inner state, same as tie breaker and heuristic
    struct FinalizingTieBreaker {
        static bool is_better(const Node& a, const Node& b);
    };
}
