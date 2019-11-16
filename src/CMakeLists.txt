cmake_minimum_required(VERSION 3.15)
project(pathp)

set(CMAKE_CXX_STANDARD 17)

add_library(pathp
    pugixml.cpp
    ioadapter.cpp
    map.cpp
    search/interface.cpp
    search/breadthfirst.cpp
    search/dijkstra.cpp
    search/astar.cpp
    search/jumppoint.cpp
    search/thetastar.cpp
)
