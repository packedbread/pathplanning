#pragma once
#include <string>
#include "../src/ioadapter.hpp"


struct IOAdapterFixture {
    planner::IOAdapter adapter;

    IOAdapterFixture(const std::string& filename = "data/test.xml");  // todo: add cmake variable for absolute path in `filename`
};
