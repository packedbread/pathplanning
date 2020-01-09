cmake_minimum_required(VERSION 3.15)
project(pathp)

set(CMAKE_CXX_STANDARD 17)

add_library(pathp STATIC
    pugixml.cpp
    ioadapter.cpp
    map.cpp
    search/interface.cpp
    search/tiebreaker.cpp
    search/astar.cpp
)
