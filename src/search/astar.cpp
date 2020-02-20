#include "astar.hpp"
#include "float_comparison.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <set>

namespace {
    using namespace planner;

    using NodePtr = std::shared_ptr<Node>;

    struct NodePtrComparator {
        double heuristic_weight;
        const TieBreaker& tie_breaker;

        NodePtrComparator(double heuristic_weight, const TieBreaker& tie_breaker) :
            heuristic_weight(heuristic_weight),
            tie_breaker(tie_breaker)
            {}

        // <less> comparator: a < b
        [[nodiscard]] bool operator () (const NodePtr& a, const NodePtr& b) const {
            auto a_cumulative = a->distance + heuristic_weight * a->estimation;
            auto b_cumulative = b->distance + heuristic_weight * b->estimation;
            if (very_close_equals(a_cumulative, b_cumulative)) {
                return tie_breaker(*a, *b);
            }
            return a_cumulative < b_cumulative;
        }
    };

    struct SearchSpace {
        std::set<NodePtr, NodePtrComparator> storage;
        std::unordered_map<Point, decltype(storage)::const_iterator> checker;
        std::unordered_set<Point> closed;

        explicit SearchSpace(const NodePtrComparator& comparator) : storage{ comparator }, checker{} {}

        void insert(const NodePtr& node_ptr) {
            if (node_ptr) {
                checker.emplace(node_ptr->position, storage.insert(node_ptr).first);
            }
        }

        bool empty() const {
            return storage.empty();
        }

        NodePtr optimal() const {
            if (!empty()) {
                return *storage.begin();
            }
            return nullptr;
        }

        void erase_optimal() {
            checker.erase(optimal()->position);
            storage.erase(storage.begin());
        }

        void close(const Point& position) {
            closed.insert(position);
        }

        bool contains(const Point& point) const {
            return checker.find(point) != checker.end();
        }

        decltype(storage)::const_iterator find(const Point& point) const {
            auto iterator = checker.find(point);
            if (iterator == checker.end()) {
                return storage.end();
            }
            return iterator->second;
        }

        bool is_closed(const Point& point) {
            return closed.find(point) != closed.end();
        }

        void erase(const Point& position) {
            auto iterator = checker.find(position);
            if (iterator != checker.end()) {
                storage.erase(iterator->second);
                checker.erase(iterator);
            }
        }
    };

    // todo: move map movement options from `search` to `map` properties
    std::vector<std::pair<Point, double>> generate_next_positions(const Point& position, const GridMap<>& map, bool allow_diagonal, bool cut_corners, bool allow_squeeze) {
        std::vector<std::pair<int, int>> deltas = {
            { -1, -1 },
            { -1, 0 },
            { -1, 1 },
            { 0, 1 },
            { 1, 1 },
            { 1, 0 },
            { 1, -1 },
            { 0, -1 }
        };

        auto is_diagonal_delta = [](const std::pair<int, int>& delta) -> bool { return delta.first && delta.second; };
        auto is_cut_corner_move = [&map](const Point& origin, const std::pair<int, int>& delta) -> int {
            return map.bordered_at(origin.x + delta.first, origin.y) == CellType::obstacle ^
                   map.bordered_at(origin.x, origin.y + delta.second) == CellType::obstacle;
        };
        auto is_squeeze_move = [&map](const Point& origin, const std::pair<int, int>& delta) -> int {
            return map.bordered_at(origin.x + delta.first, origin.y) == CellType::obstacle &&
                   map.bordered_at(origin.x, origin.y + delta.second) == CellType::obstacle;
        };

        std::vector<std::pair<Point, double>> next_positions;
        for (const std::pair<int, int>& delta : deltas) {
            Point next{ position.x + delta.first, position.y + delta.second };
            if (map.bordered_at(next.x, next.y) == CellType::obstacle) {
                continue;
            }
            if (is_diagonal_delta(delta)) {
                if (!allow_diagonal) {
                    continue;
                }
                if (is_cut_corner_move(position, delta)) {
                    if (!cut_corners) {
                        continue;
                    }
                    if (is_squeeze_move(position, delta)) {
                        if (!allow_squeeze) {
                            continue;
                        }
                    }
                }
                next_positions.emplace_back(next, std::sqrt(2));  // todo: move `sqrt(2)` and `1` logic to metric distance determination
            } else {
                next_positions.emplace_back(next, 1);
            }
        }
        return next_positions;
    }

    void expand(
        const NodePtr& optimal,
        const GridMap<>& map,
        const Heuristic<Point>& heuristic,
        const Point& destination,
        SearchSpace& search_space,
        bool allow_diagonal,
        bool cut_corners,
        bool allow_squeeze
    ) {
        std::vector<std::pair<Point, double>> next_positions = generate_next_positions(optimal->position, map, allow_diagonal, cut_corners, allow_squeeze);
        for (auto&& [point, distance] : next_positions) {
            if (search_space.is_closed(point)) {
                continue;
            }
            double candidate_distance = optimal->distance + distance;
            if (search_space.contains(point)) {
                NodePtr existing = *search_space.find(point);
                if (candidate_distance < existing->distance) {
                    search_space.erase(existing->position);
                    auto new_node = std::make_shared<Node>(point, candidate_distance, heuristic(point, destination), optimal);
                    search_space.insert(new_node);
                }
            } else {
                auto new_node = std::make_shared<Node>(point, candidate_distance, heuristic(point, destination), optimal);
                search_space.insert(new_node);
            }
        }
    }
}


namespace planner {
    SearchState AStar::search(Point from, Point to, const GridMap<CellType>& map) const {
        auto start_time = std::chrono::high_resolution_clock::now();

        NodePtrComparator comparator{ options.heuristic_weight, *tie_breaker };
        SearchState state;
        SearchSpace search_space{ comparator };
        auto start = std::make_shared<Node>(from, 0, (*heuristic)(from, to), nullptr);
        search_space.insert(start);

        while (!search_space.empty()) {
            NodePtr optimal = search_space.optimal();
            if (optimal->position == to) {
                state.path_found = true;
                break;
            }
            search_space.erase_optimal();
            search_space.close(optimal->position);
            expand(optimal, map, *heuristic, to, search_space, options.allow_diagonal, options.cut_corners, options.allow_squeeze);
            state.closed.push_back(optimal);
        }

        if (state.path_found) {
            auto optimal = search_space.optimal();
            while (optimal != nullptr) {
                state.path.push_front(optimal);
                optimal = optimal->expanded_from;
            }
            state.open = { std::begin(search_space.storage), std::end(search_space.storage) };
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        state.time_spent = end_time - start_time;

        return state;
    }
}
