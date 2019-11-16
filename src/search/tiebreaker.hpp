#pragma once


namespace planner {
    template <typename Node>  // todo: this probably should not be template
    struct TieBreaker{
        virtual bool is_worse(const Node& a, const Node& b) const = 0;
    };

    template <typename Node>
    struct GMax : TieBreaker<Node> {
        bool is_worse(const Node &a, const Node &b) const override {
            return false;  // todo: implement gmax tie breaker
        }
    };

    template <typename Node>
    struct GMin : TieBreaker<Node> {
        bool is_worse(const Node &a, const Node &b) const override {
            return false;  // todo: implement gmin tie breaker
        }
    };
}
