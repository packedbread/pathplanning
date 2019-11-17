#include <cstddef>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "ioadapter.hpp"

namespace planner {
    IOAdapter::IOAdapter(const std::string &filename) {
        std::ifstream input{filename};
        document.load(input);
        input.close();
    }

    void IOAdapter::save_document(const std::string &filename) const {
        std::ofstream output(filename);
        document.save(output);
        output.close();
    }

    GridMap<CellType> IOAdapter::read_map() const {
        auto map_node = document.child("root").child("map");
        size_t width = static_cast<size_t>(std::stoull(map_node.child_value("width")));
        size_t height = static_cast<size_t>(std::stoull(map_node.child_value("height")));
        double cell_size = static_cast<double>(std::stod(map_node.child_value("cellsize")));
        std::vector<int> data;
        data.reserve(width * height);
        for (const auto &row : map_node.child("grid").children()) {
            std::stringstream row_stream(row.first_child().value());  // todo: parse row contents as whole string, instead of just parsing the first line
            for (size_t i = 0; i < width; ++i) {
                std::cerr << row.value();
                if (std::string token; std::getline(row_stream, token, ' ')) {
                    data.push_back(std::stoi(token));
                } else {
                    throw std::logic_error{ "not enough tokens in a row" };
                }
            }
        }
        return {width, height, cell_size, data};
    }

    std::pair<Point, Point> IOAdapter::read_locations() const {
        auto map_node = document.child("root").child("map");
        Point start = {
                static_cast<size_t>(std::stoull(map_node.child_value("startx"))),
                static_cast<size_t>(std::stoull(map_node.child_value("starty"))),
        };
        Point finish = {
                static_cast<size_t>(std::stoull(map_node.child_value("finishx"))),
                static_cast<size_t>(std::stoull(map_node.child_value("finishy"))),
        };
        return { start, finish };
    }

    std::shared_ptr<Heuristic<Point>> parse_metric(const std::string& text) {
        if (text == "diagonal") {
            return std::make_unique<Diagonal<Point>>();
        } else if (text == "manhattan") {
            return std::make_unique<Manhattan<Point>>();
        } else if (text == "euclidean") {
            return std::make_unique<Euclidean<Point>>();
        } else if (text == "chebyshev") {
            return std::make_unique<Chebyshev<Point>>();
        }
        throw std::logic_error{ "unknown heuristic type: " + text };
    }

    std::shared_ptr<TieBreaker> parse_tie_breaker(const std::string& text) {
        if (text == "g-max") {
            return std::make_unique<GMax>();
        } else if (text == "g-min") {
            return std::make_unique<GMin>();
        }
        throw std::logic_error{ "unknown tie breaker: " + text };
    }

    std::shared_ptr<Search> IOAdapter::read_algorithm() const {
        auto algorithm_node = document.child("root").child("algorithm");
        auto heuristic = parse_metric(algorithm_node.child_value("metrictype"));
        auto tie_breaker = parse_tie_breaker(algorithm_node.child_value("breakingties"));
        Options options{
            std::stod(algorithm_node.child_value("hweight")),
            std::string{algorithm_node.child_value("allowdiagonal")} == "true",
            std::string{algorithm_node.child_value("cutcorners")} == "true",
            std::string{algorithm_node.child_value("allowsqueeze")} == "true",
        };
        std::string search_type = algorithm_node.child_value("searchtype");
        if (search_type == "bfs") {
            return std::make_unique<BreadthFirst>(std::move(heuristic), std::move(tie_breaker), options);
        } else if (search_type == "dijkstra") {
            return std::make_unique<Dijkstra>(std::move(heuristic), std::move(tie_breaker), options);
        } else if (search_type == "astar") {
            return std::make_unique<AStar>(std::move(heuristic), std::move(tie_breaker), options);
        } else if (search_type == "jp_search") {
            return std::make_unique<JumpPoint>(std::move(heuristic), std::move(tie_breaker), options);
        } else if (search_type == "theta") {
            return std::make_unique<ThetaStar>(std::move(heuristic), std::move(tie_breaker), options);
        }
        throw std::logic_error{ "unknown search type: " + search_type };
    }
}
