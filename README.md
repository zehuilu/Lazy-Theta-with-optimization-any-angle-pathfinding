# Lazy Theta* with optimization pathfinding
This is a customized version of Lazy-Theta-with-optimization-any-angle-pathfinding. Given a map and a set of starts and goals, this algorithm can return an optimal path. This repo has very easy-to-build-and-use C++ implementation and Python wrapper.


This repo has been tested with:
* GCC 13.3.0, CMake 3.28.3, Ubuntu 24.04.3 LTS
* GCC 10.2.0, CMake 3.16.3, Ubuntu 20.04.2 LTS
* GCC 9.3.0, CMake 3.16.3, Ubuntu 20.04.1 LTS
* Clang 15.0.0, CMake 3.27.7, macOS 13.6.6
* Clang 12.0.0, CMake 3.18.3, macOS 10.15.7
* Clang 12.0.0, CMake 3.19.3, macOS 11.1

Dependencies
============
For Python:
* [pybind11](https://github.com/pybind/pybind11) If you only install `pybind11` by `pip`, it's possible that CMake can't find it. But you can install it by `apt` or `brew`.
* [numpy](https://numpy.org/).
* [matplotlib](https://matplotlib.org/).


Build
=====
```bash
$ sudo apt install gcc g++ cmake libtbb-dev # For macOS: brew install tbb
$ sudo apt install python3-pybind11 # For macOS: brew install pybind11


$ pip3 install numpy matplotlib
$ git clone https://github.com/zehuilu/Lazy-Theta-with-optimization-any-angle-pathfinding.git
$ cd <MAIN_DIRECTORY>
$ cd build
$ cmake .. -DPYTHON_EXECUTABLE=$(which python3)
$ make
```


If you want to use [conda](https://docs.conda.io/projects/conda/en/latest/index.html) to manage environment:
```bash
$ conda create --name lazy python numpy matplotlib
$ conda activate lazy
$ conda install conda-forge::pybind11

$ git clone https://github.com/zehuilu/astar-algorithm-cpp.git
$ cd <MAIN_DIRECTORY>
$ cd build
$ cmake ..  # For debug mode, run: cmake .. -DCMAKE_BUILD_TYPE=Debug
$ make
```

Note: If you are using conda, make sure you are in the right env `lazy` before you use cmake and make to compile.
Otherwise, conda will link to the Python associated with the base environment or other environments, which are wrong.




Usage
=====

For C++, the main file is `src/main_single_path.cpp`.
```
$ cd <MAIN_DIRECTORY>
$ build/main_single_path
```

For Python, the main file is `test/test_LazyThetaStarPython.py`.
```
$ cd <MAIN_DIRECTORY>
$ python3 test/test_LazyThetaStarPython.py
```

Or `test/test_solver_and_plot.py`.
```
$ cd <MAIN_DIRECTORY>
$ python3 test/test_solver_and_plot.py
```

For Python, to run `LazyThetaStarPython.FindPathMany()` with Intel TBB (multi-threading), see `example/run_FindPathAllTBB.py`.
The function `LazyThetaStarPython.FindPathManyTBB()` is used in line 36, and the usage is the same as `LazyThetaStarPython.FindPathMany()`.
```
$ cd <MAIN_DIRECTORY>
$ python3 example/run_FindPathAllTBB.py
```


Example
=======

**Python**

To call the Lazy Theta Star solver in Python, a simple example is shown below. More details are in `test/test_solver_and_plot.py` and `test/test_LazyThetaStarPython.py`.

```python
import LazyThetaStarPython
map_width = 20
map_height = 20
# world_map is a 1D list (row-major), 0 means no obstacles, 255 means blocked by obstacles
start = [5, 8] # coordinates for start
goal = [35, 34] # coordinates for goal
# solve it
path_single, distance_single = LazyThetaStarPython.FindPath(start, goal, world_map, map_width, map_height)
```

Run `test/test_solver_and_plot.py`, the result is shown below. Time used is 0.55 ms.
![single path](doc/path_single.png?raw=true "Single Path")


For a nice animation, run
```
cd <MAIN_DIRECTORY>
python3 test/test_solve_plot_one_by_one.py
```

```
cd <MAIN_DIRECTORY>
python3 test/test_solve_plot_one_by_one_many.py
```


**A frequent issue after compilation**

If after compilation, you found this error (even if you set up the path for `LazyThetaStarPython` correctly)
```
    import LazyThetaStarPython
ModuleNotFoundError: No module named 'LazyThetaStarPython'
```

This problem is caused by the inconsistency between the python version of pybind11 and the python version of the local environment. Please refer to this [page](https://pybind11.readthedocs.io/en/stable/faq.html?highlight=cmake#cmake-doesn-t-detect-the-right-python-version).

My solution is to use
```
cmake .. -DPYTHON_EXECUTABLE=$(which python3)
```
to make cmake be aware of the default python3 version. This is also the default python3 version for pybind11.


**C++**

To call the Lazy Theta Star solver in C++, a simple example is shown below. More details are in `src/main_single_path.cpp`.

```c++
// ignore all the headers, see more details in src/main_single_path.cpp
int mapSizeX = 70; // width
int mapSizeY = 20; // length
int start[2] = {1, 1};
int end[2] = {68, 18};
// Map_1D is a std::vector<int>, 0 means no obstacles, 255 means blocked by obstacles
// solve it
// this is a tuple: std::tuple<std::vector<int>, float>
auto [path, distance] = find_path(start, end, Map_1D, mapSizeX, mapSizeY);
```


Run `src/main_single_path.cpp`, the result is shown on the console. Time used is 0.697 ms.
```
######################################################################
#S   #              #                                                #
#    #              #                                                #
#    #              #                                                #
#    #              #                            3                  4#
#    #              #                             ################## #
#    #              #                                           #    #
#    #              #                                           #5   #
#    #              #                                           # ####
#    #              #                                           #    #
#    #              #                                           #    #
#    #              #                                           #    #
#    #              #                                           #    #
#    #              #                                           #    #
#   1               #                                           #    #
#                   #                                           #6   #
#                                 2                             #    #
#                                                               #    #
#                                                               #   E#
######################################################################
#  = walls
S  = start
E  = end
number = path nodes
```
