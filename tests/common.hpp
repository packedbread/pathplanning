#pragma once
#include "../src/ioadapter.hpp"


struct IOAdapterFixture {
    planner::IOAdapter adapter;

    IOAdapterFixture() : adapter("data/test.xml") {}  // todo: add cmake variable for absolute path in `filename`
};
