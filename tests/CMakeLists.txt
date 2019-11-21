cmake_minimum_required(VERSION 3.15)
project(tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(COPY data DESTINATION .)

add_executable(tests main.cpp common.cpp test_ioadapter.cpp test_map.cpp test_quadratic.cpp test_functional.cpp)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost COMPONENTS unit_test_framework)
if(!Boost_FOUND)
    set(Boost_USE_STATIC_LIBS OFF)
    find_package(Boost REQUIRED COMPONENTS unit_test_framework)
endif()
# todo: review use cases of static and shared boost libs, because appveyor by default has only static and apt-get installs shared
include_directories(${Boost_INCLUDE_DIR})
target_link_libraries(tests ${Boost_LIBRARIES})

target_link_libraries(tests pathp)
