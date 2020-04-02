# Path Planning

[![Build Status](https://travis-ci.com/packedbread/pathplanning.svg?branch=master)](https://travis-ci.com/packedbread/pathplanning)
[![Build status](https://ci.appveyor.com/api/projects/status/94tqu36hlpy5i23d/branch/master?svg=true)](https://ci.appveyor.com/project/packedbread/pathplanning/branch/master)

Implementation of path planning algorithms. Given an xml document, this program can find a path from one point to another and output it into an xml document. Several algorithms are supported with a number of heuristics and options.

## Build
### Requirements:
- c++ compiler with c++17 support, including std::filesystem, for example gcc-9
- cmake >= 3.15
- boost >= 1.71.0
  - only unit_test_framework component is required


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


## Input format
Input is in XML format. Example:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<root>
    <map> <!--map parameters-->
        <width>3</width> <!--Width of the map-->
        <height>1</height> <!--Height of the map-->
        <cellsize>1</cellsize> <!--Size of single item in the row-->
        <startx>0</startx> <!--Start position x coordinate-->
        <starty>0</starty> <!--Start position y coordinate-->
        <finishx>1</finishx> <!--Finish position x coordinate-->
        <finishy>0</finishy> <!--Finish position y coordinate-->
        <grid> <!--Map representation-->
            <row>0 0 1</row> <!--Row values, allowed values: 0 - clear, 1 - obstacle-->
        </grid>
    </map>
    <algorithm> <!--Algorithm options-->
        <searchtype>astar</searchtype> <!--Type of the algorithm, allowed values are: dijkstra, astar-->
        <metrictype>euclid</metrictype> <!--Heuristic type, allowed values are: diagonal, euclid, manhattan, chebyshev-->
        <breakingties>g-max</breakingties> <!--Tie breaker type, allowed values are: g-max, g-min-->
        <hweight>1</hweight> <!--Heuristic weight in distance estimation calculation, allowed values: floating point values-->
        <allowdiagonal>true</allowdiagonal> <!--Are diagonal moves allowed, allowed values: true, false-->
        <cutcorners>true</cutcorners> <!--Is corner cutting allowed, allowed values: true, false-->
        <allowsqueeze>true</allowsqueeze> <!--Is squeezing allowed, allowed values: true, false-->
    </algorithm>
    <options> <!--Program options-->
        <loglevel>1</loglevel> <!--Logging verbosity, allowed values are 0, 0.5, 1, 1.5, 2-->
        <logpath></logpath> <!--Log directory-->
        <logfilename></logfilename> <!--Log file name-->
    </options>
</root>
```
Other examples can be found in tests.

## Output format
Output is also in XML format. It will contain all of the input and several other tags. Example of added tags:
```xml
<root>
    <log> <!--Program output information-->
        <summary numberofsteps="350" nodescreated="400" length="29.384777" length_scaled="705.23464965820312" time="0.000000"/> <!--Summary of the result, will contain numberofsteps (number of expanded nodes during the search), nodescreated (number of created nodes during the search), length (distance of the found path), length_scaled (length multiplied by cell size), time (search time)-->
        <path> <!--Path on the map-->
            <row number="0">* * 1</row> <!--'*' character marks the path-->
        </path>
        <lplevel/> <!--Path nodes info-->
        <hplevel/> <!--Path section info-->
        <lowlevel/> <!--Open and Closed lists history info-->
    </log>
</root>
```
For examples of output format refer to tests.

## Supported heuristics
There are currently 4 supported heuristics: 
- diagonal
- euclid
- manhattan
- chebyshev

Assuming `dx` and `dy` are absolute values of distance difference between starting point and goal point, the heuristics are calculated in the following way:
- `diagonal = |dx - dy| + sqrt(2) * min(dx, dy)`
- `euclid = sqrt(dx^2 + dy^2)`
- `manhattan = dx + dy`
- `chebyshev = max(dx, dy)`

### Heuristic weight
Parameter `hweight` in the input document can be used to adjust the weight of heuristic estimation in the total distance estimation. Concretly:

`Total distance estimation = actual distance + hweight * heuristic estimate`

So by changing the value of `hweight` parameter you can adjust the importance of the heuristic in distance estimation.

Recommended value is 1, it is also the default value. It gives highest speed while providing strong guarantees on the result. Values from 0 to 1 will still provide the same guarantees for the resulting path, but will expand more nodes during the search, leading to higher memory and time usage. 

### Tie breakers
Currently there are two tie breakers:
- `g-max`
- `g-min`

`g-max` tie breaker would prefer node with higher actual distance. `g-min` - with higher heurstic estimate.

For more information on heuristic weights and tie breakers refer to this comprehensive guide page:
http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
