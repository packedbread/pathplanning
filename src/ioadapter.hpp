#pragma once
#include <memory>
#include <utility>
#include "pugixml.hpp"
#include "map.hpp"
#include "search/search.hpp"


namespace planner {
    class IOAdapter {
        pugi::xml_document document;
    public:
        IOAdapter(const std::string& filename);

        void save_document(const std::string& filename) const;

        [[nodiscard]] GridMap<CellType> read_map() const;
        [[nodiscard]] std::pair<Point, Point> read_locations() const;
        [[nodiscard]] std::shared_ptr<Search> read_algorithm() const;
    };
}
