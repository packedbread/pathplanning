cmake_minimum_required(VERSION 3.15)
project(path_planning)

set(CMAKE_CXX_STANDARD 17)

add_executable(path_planning src/main.cpp)

if (${MINGW})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -static -static-libgcc -static-libstdc++")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

add_subdirectory(src)
add_subdirectory(tests)
target_link_libraries(path_planning pathp)
