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
        IOAdapter(std::istream& stream);

        void save_document(const std::string& filename) const;
        void save_document(std::ostream& output) const;

        [[nodiscard]] GridMap<CellType> read_map() const;
        [[nodiscard]] std::pair<Point, Point> read_locations() const;
        [[nodiscard]] std::shared_ptr<Search> read_algorithm() const;
        [[nodiscard]] double read_path_length() const;  // todo: replace this with reading full log node

        void write_result(const SearchState& result, const GridMap<CellType>& map);  // todo: remove map parameter
    };
}
