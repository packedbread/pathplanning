#include <cstddef>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unordered_set>

#include "ioadapter.hpp"


namespace planner {
    IOAdapter::IOAdapter(const std::string &filename) {
        std::ifstream input{filename};
        document.load(input);
        input.close();
    }

    IOAdapter::IOAdapter(std::istream& stream) {
        document.load(stream);
    }

    void IOAdapter::save_document(const std::string& filename) const {
        std::ofstream output(filename);
        document.save(output);
        output.close();
    }

    void IOAdapter::save_document(std::ostream& output) const {
        document.save(output);
    }

    std::string extract_value_with_default(const pugi::xml_node& node, const std::string& name, const std::string& default_value) {
        if (auto child_node = node.child(name.c_str()); child_node) {
            return child_node.first_child().value();
        }
        return default_value;
    }

    GridMap<CellType> IOAdapter::read_map() const {
        auto map_node = document.child("root").child("map");
        size_t width = static_cast<size_t>(std::stoull(map_node.child_value("width")));
        size_t height = static_cast<size_t>(std::stoull(map_node.child_value("height")));
        double cell_size = static_cast<double>(std::stod(extract_value_with_default(map_node, "cellsize", "1.0")));
        std::vector<int> data;
        data.reserve(width * height);
        for (const auto &row : map_node.child("grid").children()) {
            std::stringstream row_stream(row.first_child().value());  // todo: parse row contents as whole string, instead of just parsing the first line
            for (size_t i = 0; i < width; ++i) {
                if (std::string token; std::getline(row_stream, token, ' ')) {
                    data.push_back(std::stoi(token));
                } else {
                    throw std::logic_error{ "not enough tokens in a row" };
                }
            }
        }
        return { width, height, cell_size, data, InverseMapper{} };
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

    bool parse_bool_value(const std::string& text) {
        return !(text == "false" || text == "False" || text == "0" || text == "0.0");
    }

    std::shared_ptr<Search> IOAdapter::read_algorithm() const {
        auto algorithm_node = document.child("root").child("algorithm");
        auto heuristic = parse_metric(extract_value_with_default(algorithm_node, "metrictype", "euclidean"));
        auto tie_breaker = parse_tie_breaker(extract_value_with_default(algorithm_node, "breakingties", "g-max"));
        Options options{
            std::stod(extract_value_with_default(algorithm_node, "hweight", "1.0")),
            parse_bool_value(extract_value_with_default(algorithm_node, "allowdiagonal", "true")),
            parse_bool_value(extract_value_with_default(algorithm_node, "cutcorners", "true")),
            parse_bool_value(extract_value_with_default(algorithm_node, "allowsqueeze", "true")),
        };
        std::string search_type = algorithm_node.child_value("searchtype");
        if (search_type == "astar") {
            return std::make_unique<AStar>(std::move(heuristic), std::move(tie_breaker), options);
        } else if (search_type == "dijkstra") {
            options.heuristic_weight = 0;
            return std::make_unique<AStar>(std::move(heuristic), std::move(tie_breaker), options);
        }
//        if (search_type == "bfs") {
//            return std::make_unique<BreadthFirst>(std::move(heuristic), std::move(tie_breaker), options);
//        } else if (search_type == "jp_search") {
//            return std::make_unique<JumpPoint>(std::move(heuristic), std::move(tie_breaker), options);
//        } else if (search_type == "theta") {
//            return std::make_unique<ThetaStar>(std::move(heuristic), std::move(tie_breaker), options);
//        }
        throw std::logic_error{ "unknown search type: " + search_type };
    }

    void IOAdapter::write_result(const SearchState& result, const GridMap<CellType>& map) {
        auto root_node = document.child("root");
        while (root_node.find_node([](const pugi::xml_node& p) { return std::string{ p.name() } == "log"; })) {
            root_node.remove_child("log");
        }
        auto log_node = root_node.append_child("log");

        auto mapfilename = log_node.append_child("mapfilename");

        auto summary_node = log_node.append_child("summary");
        summary_node.append_attribute("numberofsteps") = result.closed.size() + (result.path.empty() ? 0 : 1);
        summary_node.append_attribute("nodescreated") = result.closed.size() + result.open.size();
        summary_node.append_attribute("length") = result.path_length();
        summary_node.append_attribute("length_scaled") = result.path_length() * map.get_cell_size();
        summary_node.append_attribute("time") = std::chrono::duration_cast<std::chrono::nanoseconds>(result.time_spent).count() / 1e9;

        if (map.get_width() != 0 && map.get_height() != 0) {
            auto path_node = log_node.append_child("path");
            std::unordered_set<Point> path_points;
            for (const auto& ptr : result.path) {
                path_points.insert(ptr->position);
            }
            for (size_t y = 0; y < map.get_height(); ++y) {
                auto row_node = path_node.append_child("row");
                row_node.append_attribute("number") = y;
                std::string row_value;
                if (path_points.find({ 0, y }) == std::end(path_points)) {
                    row_value += map(0, y) == CellType::empty ? "0" : "1";
                } else {
                    row_value += "*";
                }
                for (size_t x = 1; x < map.get_width(); ++x) {
                    if (path_points.find({ x, y }) == std::end(path_points)) {
                        row_value += map(x, y) == CellType::empty ? " 0" : " 1";
                    } else {
                        row_value += " *";
                    }
                }
                row_node.append_child(pugi::node_pcdata).set_value(row_value.c_str());
            }
        }

        auto lplevel = log_node.append_child("lplevel");
        auto hplevel = log_node.append_child("hplevel");
    }

    double IOAdapter::read_path_length() const {
        auto summary_node = document.child("root").child("log").child("summary");
        return std::stod(summary_node.attribute("length").value());
    }
}
