cmake_minimum_required(VERSION 3.15)
project(tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(COPY data DESTINATION .)

add_executable(tests main.cpp common.cpp test_ioadapter.cpp test_map.cpp test_quadratic.cpp test_functional.cpp)

find_package(Boost REQUIRED COMPONENTS unit_test_framework)
include_directories(${Boost_INCLUDE_DIR})
target_link_libraries(tests ${Boost_LIBRARIES})

target_link_libraries(tests pathp)

target_link_libraries(tests stdc++fs)
