#include <boost/test/unit_test.hpp>
#include "../src/quadratic.hpp"


using namespace planner;

BOOST_AUTO_TEST_SUITE(quadratic)

BOOST_AUTO_TEST_CASE(test_constructors) {
    {
        Quadratic<int> q1;
        BOOST_CHECK_EQUAL(q1.real, 0);
        BOOST_CHECK_EQUAL(q1.imaginary, 0);
    }
    {
        Quadratic<int> q2{1, 2};
        BOOST_CHECK_EQUAL(q2.real, 1);
        BOOST_CHECK_EQUAL(q2.imaginary, 2);
    }
}

BOOST_AUTO_TEST_CASE(test_arithmetic) {
    Quadratic<int> q1{ 1, 2 };
    Quadratic<int> q2{ 3, -5};
    q1 += q2;
    BOOST_CHECK_EQUAL(q1,  Quadratic<int>(4, -3));
    auto q3 = q1 + q2;
    BOOST_CHECK_EQUAL(q3, Quadratic<int>(7, -8));
    q3 -= q2;
    BOOST_CHECK_EQUAL(q3,  Quadratic<int>(4, -3));
    auto q4 = q3 - q2;
    BOOST_CHECK_EQUAL(q4,  Quadratic<int>(1, 2));
}

BOOST_AUTO_TEST_CASE(test_evaluation) {
    {
        Quadratic<int> q{ 1, 1 };
        BOOST_CHECK_CLOSE(evaluate(q), 1 + sqrt(2), 1e-9);
    }
    {
        Quadratic<int> q{ 0, 1 };
        BOOST_CHECK_CLOSE(evaluate(q), sqrt(2), 1e-9);
    }
    {
        Quadratic<int> q{ 1, 0 };
        BOOST_CHECK_CLOSE(evaluate(q), 1, 1e-9);
    }
    {
        Quadratic<int> q{ 5, -4 };
        BOOST_CHECK_CLOSE(evaluate(q), 5 - 4 * sqrt(2), 1e-9);
    }
}

BOOST_AUTO_TEST_SUITE_END()