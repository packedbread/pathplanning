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
    auto result = search->search(locations.first, locations.second, map);
    adapter.write_result(result, map);
    if (argc < 2) {
        adapter.save_document(std::cout);
    } else {
        std::string input_filename{ argv[1] };
        if (input_filename.size() >= 4 && input_filename.substr(input_filename.size() - 4) == std::string{ ".xml" }) {
            adapter.save_document(input_filename.substr(0, input_filename.size() - 4) + "_log.xml");
        } else {
            adapter.save_document(std::cout);
        }
    }

    return 0;
}
