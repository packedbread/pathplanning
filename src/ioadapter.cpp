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
        } else if (text == "euclidean" || text == "euclid") {
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

    void write_node(pugi::xml_node& parent_node, const std::shared_ptr<Node>& node) {
        auto child_node = parent_node.append_child("node");
        child_node.append_attribute("x").set_value(node->position.x);
        child_node.append_attribute("y").set_value(node->position.y);
        child_node.append_attribute("F").set_value(node->distance);
        child_node.append_attribute("g").set_value(node->estimation);
        if (node->expanded_from != nullptr) {
            child_node.append_attribute("parent_x").set_value(node->expanded_from->position.x);
            child_node.append_attribute("parent_y").set_value(node->expanded_from->position.y);
        }
    }

    void write_step(pugi::xml_node& lowlevel_node, size_t step, const std::list<std::shared_ptr<Node>>& open, const std::list<std::shared_ptr<Node>>& closed) {
        auto step_node = lowlevel_node.append_child("step");
        step_node.append_attribute("number").set_value(step);
        auto open_node = step_node.append_child("open");
        for (const auto& node : open) {
            write_node(open_node, node);
        }
        auto close_node = step_node.append_child("close");
        for (const auto& node : closed) {
            write_node(close_node, node);
        }
    }

    void IOAdapter::write_result(const SearchState& result, std::string input_filename, const GridMap<CellType>& map, const LogOptions& log_options) {
        auto root_node = document.child("root");
        while (root_node.find_node([](const pugi::xml_node& p) { return std::string{ p.name() } == "log"; })) {
            root_node.remove_child("log");
        }
        auto log_node = root_node.append_child("log");

        log_node.append_child("mapfilename").set_value(input_filename.data());

        auto summary_node = log_node.append_child("summary");
        if (log_options.is_level_at_least_tiny()) {
            summary_node.append_attribute("numberofsteps") = result.closed.size() + (result.path.empty() ? 0 : 1);
            summary_node.append_attribute("nodescreated") = result.closed.size() + result.open.size();
            summary_node.append_attribute("length") = result.path_length();
            summary_node.append_attribute("length_scaled") = result.path_length() * map.get_cell_size();
            summary_node.append_attribute("time") = std::chrono::duration_cast<std::chrono::nanoseconds>(result.time_spent).count() / 1e9;
        }

        if (log_options.is_level_at_least_short() && map.get_width() != 0 && map.get_height() != 0) {
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
        if (log_options.is_level_at_least_short()) {
            size_t number = 0;
            for (const auto& path_node : result.path) {
                auto node = lplevel.append_child("node");
                node.append_attribute("x").set_value(path_node->position.x);
                node.append_attribute("y").set_value(path_node->position.y);
                node.append_attribute("number").set_value(number);
                ++number;
            }
        }
        auto hplevel = log_node.append_child("hplevel");
        if (log_options.is_level_at_least_short()) {
            if (result.path.size() == 1) {
                const auto& path_node = result.path.front();
                pugi::xml_node section = hplevel.append_child("section");
                section.append_attribute("number").set_value(0);
                section.append_attribute("start.x").set_value(path_node->position.x);
                section.append_attribute("start.y").set_value(path_node->position.y);
                section.append_attribute("finish.x").set_value(path_node->position.x);
                section.append_attribute("finish.y").set_value(path_node->position.y);
                section.append_attribute("length").set_value(0);
            } else if (result.path.size() > 1) {
                size_t section_number = 0;
                auto prev_path_node = result.path.front();
                pugi::xml_node section = hplevel.append_child("section");
                section.append_attribute("number").set_value(section_number);
                section.append_attribute("start.x").set_value(prev_path_node->position.x);
                section.append_attribute("start.y").set_value(prev_path_node->position.y);
                auto cit = std::next(std::begin(result.path));
                int dx = static_cast<int>((*cit)->position.x - prev_path_node->position.x);
                int dy = static_cast<int>((*cit)->position.y - prev_path_node->position.y);
                double current_path_length = 0.0;
                for (; cit != std::end(result.path); ++cit) {
                    int current_dx = static_cast<int>((*cit)->position.x - prev_path_node->position.x);
                    int current_dy = static_cast<int>((*cit)->position.y - prev_path_node->position.y);
                    if (current_dx == dx && current_dy == dy) {
                        current_path_length += std::sqrt(dx * dx + dy * dy);
                    } else {
                        section.append_attribute("finish.x").set_value(prev_path_node->position.x);
                        section.append_attribute("finish.y").set_value(prev_path_node->position.y);
                        section.append_attribute("length").set_value(current_path_length);
                        ++section_number;
                        section = hplevel.append_child("section");
                        section.append_attribute("number").set_value(section_number);
                        section.append_attribute("start.x").set_value(prev_path_node->position.x);
                        section.append_attribute("start.y").set_value(prev_path_node->position.y);
                        dx = current_dx;
                        dy = current_dy;
                        current_path_length = std::sqrt(dx * dx + dy * dy);
                    }
                    prev_path_node = *cit;
                }
                section.append_attribute("finish.x").set_value(prev_path_node->position.x);
                section.append_attribute("finish.y").set_value(prev_path_node->position.y);
                section.append_attribute("length").set_value(current_path_length);
            }
        }

        size_t step_counter = 0;
        auto lowlevel = log_node.append_child("lowlevel");
        if (log_options.is_level_at_least_full()) {
            for (auto it_open = result.open_history.begin(), it_closed = result.closed_history.begin();
                it_open != result.open_history.end() && it_closed != result.closed_history.end();
                ++it_open, ++it_closed, ++step_counter) {
                write_step(lowlevel, step_counter, *it_open, *it_closed);
            }
        }

        if (log_options.is_level_at_least_medium()) {
            write_step(lowlevel, step_counter, result.open, result.closed);
        }
    }

    double IOAdapter::read_path_length() const {
        auto summary_node = document.child("root").child("log").child("summary");
        return std::stod(summary_node.attribute("length").value());
    }

    LogOptions IOAdapter::read_log_options() const {
        LogOptions log_options;
        auto options_node = document.child("root").child("options");
        log_options.log_level = extract_value_with_default(options_node, "loglevel", "1");
        log_options.log_path = extract_value_with_default(options_node, "logpath", "");
        log_options.log_filename = extract_value_with_default(options_node, "logfilename", "");
        return log_options;
    }
}
