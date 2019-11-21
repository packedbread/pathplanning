#include <boost/test/unit_test.hpp>
#include <boost/test/data/test_case.hpp>
#include <boost/test/data/monomorphic.hpp>
#include <filesystem>
#include <iterator>
#include "common.hpp"


struct FunctionalTestDataset {
    static const std::filesystem::path tests_directory;

    using iterator = std::filesystem::directory_iterator;

    [[nodiscard]] iterator begin() const {
        return std::filesystem::begin(iterator{ tests_directory });
    }

    [[nodiscard]] iterator end() const {
        return std::filesystem::end(iterator{ tests_directory });
    }

    boost::unit_test::data::size_t size() const {
        return std::distance(begin(), end());
    }
};

namespace boost::unit_test::data::monomorphic {
    template <>
    struct is_dataset<FunctionalTestDataset> : boost::mpl::true_ {};
}

const std::filesystem::path FunctionalTestDataset::tests_directory{ "data/functional/" };


BOOST_AUTO_TEST_SUITE(functional)

BOOST_DATA_TEST_CASE(dataset, FunctionalTestDataset{}, directory_entry) {
    if (!directory_entry.is_regular_file()) {
        return;
    }
    IOAdapterFixture fixture{ directory_entry.path().u8string() };
    auto map = fixture.adapter.read_map();
    auto locations = fixture.adapter.read_locations();
    auto search = fixture.adapter.read_algorithm();
    auto result = search->search(locations.first, locations.second, map);
    double expected_length = fixture.adapter.read_path_length();
    BOOST_CHECK_CLOSE(result.path_length(), expected_length, 1e-5);
}

BOOST_AUTO_TEST_SUITE_END()
