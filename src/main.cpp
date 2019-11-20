#include <iostream>
#include <fstream>
#include <string>
#include "ioadapter.hpp"


using namespace planner;

int main(int argc, char** argv) {
    IOAdapter adapter = argc < 2 || std::string{ argv[1] } == "-" ? IOAdapter{ std::cin } : IOAdapter{ argv[1] };
    auto map = adapter.read_map();
    auto locations = adapter.read_locations();
    auto search = adapter.read_algorithm();
    auto result = search->search(locations.first, locations.second, map);
    adapter.write_result(result, map);
    if (argc < 3 || std::string{ argv[2] } == "-") {
        adapter.save_document(std::cout);
    } else {
        adapter.save_document(argv[2]);
    }

    return 0;
}