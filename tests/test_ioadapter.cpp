#include <boost/test/unit_test.hpp>
#include <iterator>
#include "common.hpp"
#include "../src/ioadapter.hpp"
#include "../src/search/interface.hpp"


using namespace planner;

BOOST_AUTO_TEST_SUITE(ioadapter)

BOOST_FIXTURE_TEST_CASE(test_read_map, IOAdapterFixture) {
    auto map = adapter.read_map();
    BOOST_CHECK_EQUAL(map.get_width(), 3);
    BOOST_CHECK_EQUAL(map.get_height(), 2);
    BOOST_CHECK_EQUAL(map.get_cell_size(), 1);
    BOOST_CHECK_EQUAL(map.get_width(), 3);
    std::vector<CellType> data = {
        CellType::empty, CellType::empty, CellType::obstacle,
        CellType::obstacle, CellType::empty, CellType::empty,
    };
    BOOST_CHECK_EQUAL_COLLECTIONS(std::begin(data), std::end(data), std::begin(map), std::end(map));
}

BOOST_FIXTURE_TEST_CASE(test_read_locations, IOAdapterFixture) {
    auto locations = adapter.read_locations();
    Point start = { 0, 0 };
    BOOST_CHECK_EQUAL(locations.first, start);
    Point finish = { 1, 1 };
    BOOST_CHECK_EQUAL(locations.second, finish);
}

BOOST_FIXTURE_TEST_CASE(test_read_algorithm, IOAdapterFixture) {
    auto algorithm = adapter.read_algorithm();
    BOOST_CHECK_NO_THROW(dynamic_cast<AStar&>(*algorithm));
    BOOST_CHECK_NO_THROW(dynamic_cast<Diagonal<Point>&>(*algorithm->get_heuristic()));
    BOOST_CHECK_NO_THROW(dynamic_cast<GMax&>(*algorithm->get_tie_breaker()));
    Options correct_options {
        1.0,
        true,
        true,
        true,
    };
    BOOST_CHECK_EQUAL(algorithm->get_options(), correct_options);
}

BOOST_AUTO_TEST_SUITE_END()
