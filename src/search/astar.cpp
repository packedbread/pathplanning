#include "astar.hpp"
#include "float_compare.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>


namespace planner {
    template <typename T, typename Container, typename Comparator>
    const Container& access_container(const std::priority_queue<T, Container, Comparator>& queue) {
        struct Access : private std::priority_queue<T, Container, Comparator> {
            static const Container& access_container(const std::priority_queue<T, Container, Comparator>& queue) {
                return queue.*&Access::c;
            }
        };
        return Access::access_container(queue);
    }

    SearchState AStar::search(Point from, Point to, const GridMap<CellType>& map) const {
        auto start_time = std::chrono::high_resolution_clock::now();

        SearchState state;
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, Comparator> open(Comparator(*this));
        auto node_ptr = std::make_shared<Node>(from, 0, (*heuristic)(from, to), nullptr);
        open.push(node_ptr);
        std::unordered_map<Point, std::shared_ptr<Node>> open_checker{ { from, node_ptr } };
        std::unordered_set<Point> closed_checker;

        auto expand = [
            &heuristic = heuristic,
            &options = options,
            &open_checker,
            &closed_checker,
            &open,
            &map,
            &to
        ](const std::shared_ptr<Node>& node) -> void {
            const auto& position = node->position;

            auto insert_point = [
                &heuristic,
                &map,
                &open,
                &open_checker,
                &closed_checker,
                &node,
                &to
            ](size_t x, size_t y, double distance_change, bool extra_condition = true) -> void {
                if (
                    map.bordered_at(x, y) != CellType::obstacle &&
                    closed_checker.find({ x, y }) == std::end(closed_checker) &&
                    extra_condition
                ) {
                    if (open_checker.find({ x, y }) == std::end(open_checker)) {
                        auto node_ptr = std::make_shared<Node>(Point{ x, y }, node->distance + distance_change, (*heuristic)({ x, y }, to), node);
                        open.push(node_ptr);
                        open_checker[{ x, y }] = node_ptr;
                    } else {
                        auto node_ptr = open_checker[{ x, y }];
                        if (node->distance + distance_change < node_ptr->distance) {
                            node_ptr->distance = node->distance + distance_change;
                            node_ptr->expanded_from = node;
                        }
                    }
                }
            };

            for (auto [dx, dy] : {std::pair<int, int>{ -1, 0 }, { 0, 1 }, { 1, 0 }, { 0, -1 }}) {
                auto x = position.x + dx;
                auto y = position.y + dy;
                insert_point(x, y, 1.0);
            }
            auto diagonal_elements = { std::pair<int, int>{ -1, -1 }, { -1, 1 }, { 1, 1 }, { 1, -1 } };
            if (options.allow_diagonal) {
                if (options.cut_corners) {
                    if (options.allow_squeeze) {
                        for (auto [dx, dy] : diagonal_elements) {
                            auto x = position.x + dx;
                            auto y = position.y + dy;
                            insert_point(x, y, std::sqrt(2));
                        }
                    } else {
                        for (auto [dx, dy] : diagonal_elements) {
                            auto x = position.x + dx;
                            auto y = position.y + dy;
                            insert_point(x, y, std::sqrt(2), map.bordered_at(position.x, y) != CellType::obstacle || map.bordered_at(x, position.y) != CellType::obstacle);
                        }
                    }
                } else {
                    for (auto [dx, dy] : diagonal_elements) {
                        auto x = position.x + dx;
                        auto y = position.y + dy;
                        insert_point(x, y, std::sqrt(2), map.bordered_at(position.x, y) != CellType::obstacle && map.bordered_at(x, position.y) != CellType::obstacle);
                    }
                }
            }
        };

        while (!open.empty()) {
            auto optimal = open.top();
            if (optimal->position == to) {
                break;
            }
            open.pop();
            expand(optimal);
            open_checker.erase(optimal->position);
            closed_checker.insert(optimal->position);
            state.closed.emplace_back(std::move(optimal));
        }

        if (!open.empty()) {
            auto optimal = open.top();
            while (optimal != nullptr) {
                state.path.push_front(optimal);
                optimal = optimal->expanded_from;
            }
            const auto& container = access_container(open);
            state.open = { std::begin(container), std::end(container) };
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        state.time_spent = end_time - start_time;

        return state;
    }
}
