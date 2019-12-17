# Path Planning

[![Build Status](https://travis-ci.com/packedbread/pathplanning.svg?branch=master)](https://travis-ci.com/packedbread/pathplanning)
[![Build status](https://ci.appveyor.com/api/projects/status/94tqu36hlpy5i23d/branch/master?svg=true)](https://ci.appveyor.com/project/packedbread/pathplanning/branch/master)

Requirements:
- c++ compiler with c++17 support, including std::filesystem
- cmake >= 3.15
- boost >= 1.71.0
  - only unit_test_framework component is required

## Build
### Debian build
1. Install cmake: https://cmake.org/download/

2. Install boost: https://www.boost.org/doc/libs/1_71_0/more/getting_started/unix-variants.html

3. Install c++ compiler with c++17 support, g++-9 for example:

    - from source: https://gcc.gnu.org/install/
    - from ppa: `sudo add-apt-repository 'ppa:ubuntu-toolchain-r/test' && sudo apt-get install g++-9`

4. Set build type and build directory:

    one of:
    - `export BUILD=Debug`
    - `export BUILD=Release`
    
    `export BUILD_DIR=${BUILD}-Build`

4. Configure cmake (while in root repo directory):

    `cmake -S . -B ${BUILD_DIR} -D CMAKE_BUILD_TYPE=${BUILD}`

5. Build:

    `cmake --build ${BUILD_DIR} --target path_planning`

6. Build tests target (optional):

    `cmake --build ${BUILD_DIR} --target tests`

7. Run tests (optional):

    `cd ${BUILD_DIR}/tests && ./tests --log_level=all`

### Windows build
1. Install cmake: https://cmake.org/download/

2. Install boost: https://www.boost.org/doc/libs/1_71_0/more/getting_started/windows.html

3. Install c++ compiler with c++17 support, MinGW (with g++-9) for example: http://www.mingw.org/

4. Set build type and boost root:

    one of:
    - `SET BUILD_TYPE=Debug`
    - `SET BUILD_TYPE=Release`
    
    `SET BOOST_ROOT=C:\Path\to\boost_1_71_0`

5. Configure cmake (while in root repo directory):
    
    `cmake -G "MinGW Makefiles" -S . -B %BUILD_TYPE%-Build -D CMAKE_BUILD_TYPE=%BUILD_TYPE%`
    
6. Build

    `cmake --build %BUILD_TYPE%-Build --target path_planning`

7. Build tests target (optional):

    `cmake --build %BUILD_TYPE%-Build --target tests`

8. Run tests (optional):

    `cd %BUILD_TYPE%-Build\tests`
    
    `tests.exe --log_level=all`


## Run executable
There are several options to run main executable, both with file input/output and without it:

1. No files, input xml file is read from standard input (stdin), resulting xml file is printed to standard output (stdout): run execuable without arguments, e.g.:

    - \*nix: `./path_planning`
    - Windows: `path_planning.exe`

2. Read from file, output to file with suffix `_log`: run with 1 argument, input filename, e.g.:

    - \*nix: `./path_planning input_file.xml`
    - Windows: `path_planning.exe input_file.xml`
    
    This will create `input_file_log.xml` as a result.
