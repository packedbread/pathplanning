#include <iostream>
#include <fstream>
#include <string>
#include "ioadapter.hpp"


using namespace planner;

int main(int argc, char** argv) {
    IOAdapter adapter = argc < 2 ? IOAdapter{ std::cin } : IOAdapter{ argv[1] };
    auto map = adapter.read_map();
    auto locations = adapter.read_locations();
    auto search = adapter.read_algorithm();
    auto log_options = adapter.read_log_options();
    auto result = search->search(locations.first, locations.second, map, log_options.is_level_at_least_full());
    adapter.write_result(result, argc < 2 ? "" : std::string{ argv[1] }, map, log_options);
    if (argc < 2) {
        adapter.save_document(std::cout);
    } else {
        std::string output_filename{ argv[1] };
        std::string slash_character = "\\";  // todo: this is really ugly
        if (output_filename.find(slash_character) == std::string::npos) {
            slash_character = "/";
        }
        if (log_options.log_path.empty() && log_options.log_filename.empty()) {
            size_t dot_position = output_filename.find_last_of('.');
            if (dot_position != std::string::npos) {
                output_filename.insert(dot_position, "_log");
            } else {
                output_filename.append("_log");
            }
        } else if (log_options.log_path.empty()) {
            output_filename = output_filename.substr(0, output_filename.rfind(slash_character));
            output_filename += log_options.log_filename;
        } else if (log_options.log_filename.empty()) {
            output_filename = output_filename.substr(output_filename.rfind(slash_character));
            output_filename = log_options.log_path + slash_character + output_filename;
            size_t dot_position = output_filename.find_last_of('.');  // todo: remove code dup
            if (dot_position != std::string::npos) {
                output_filename.insert(dot_position, "_log");
            } else {
                output_filename.append("_log");
            }
        } else {
            output_filename = log_options.log_path + slash_character + log_options.log_filename;
        }

        if (!output_filename.empty()) {
            adapter.save_document(output_filename);
        } else {
            adapter.save_document(std::cout);
        }
    }

    return 0;
}
