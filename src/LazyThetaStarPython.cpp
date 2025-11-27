#include <iostream>
#include <vector>
#include <tuple>
#include <array>
#include <chrono>
#include <math.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "tileadaptor.hpp"
#include "utility.hpp"
#include "get_combination.hpp"
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)
static constexpr float WEIGHT_PATH = 1E2;


// Multi-threaded version using Intel TBB
inline std::tuple<std::vector<std::vector<int>>, std::vector<float>> FindPathManyTBB(
    const std::vector<int> agent_position,
    const std::vector<int> targets_position,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    // release GIL for true parallelism
    pybind11::gil_scoped_release release;

    std::vector<int> start_goal_pair = get_combination(targets_position.size()/2 + 1, 2);
    size_t n_pairs = start_goal_pair.size() / 2;
    std::vector<std::vector<int>> path_many(n_pairs);
    std::vector<float> distances_many(n_pairs);

    // passing the map size, and the map
    Vectori mapSize(mapSizeX, mapSizeY);

    // Parallel version: each case builds its own Pathfinder instance
    // NOTE: for Intel CPUs, e.g., Intel® Core™ i7-13700
    // there are 8 Performance-cores and 8 Efficient-cores.
    // So the total number of threads is 2*8 + 8 = 24, which is
    // what arena(tbb::task_arena::automatic) does. (using arena.max_concurrency() = 24)
    // But I found out that when n_pairs is large, on the order of 5k+,
    // only using performance threads is faster.
    // That means we may need to manually set num_threads as 2*8 = 16.

    // TBB arena: use all cores
    tbb::task_arena arena(tbb::task_arena::automatic);

    // use all performance cores for Intel® Core™ i7-13700
    // tbb::task_arena arena(16);

    // std::cout << "num_threads (tbb::task_arena::automatic): " << arena.max_concurrency() << std::endl;

    // parallelly executed
    arena.execute([&] {
        tbb::parallel_for(size_t(0), n_pairs, [&](size_t k) {
            int start_idx = start_goal_pair[2*k];
            int goal_idx  = start_goal_pair[2*k + 1];
            int start[2], goal[2];

            if (start_idx != 0) {
                start[0] = targets_position[2*(start_idx-1)];
                start[1] = targets_position[2*(start_idx-1)+1];
            } else {
                start[0] = agent_position[0];
                start[1] = agent_position[1];
            }

            if (goal_idx != 0) {
                goal[0] = targets_position[2*(goal_idx-1)];
                goal[1] = targets_position[2*(goal_idx-1)+1];
            } else {
                goal[0] = agent_position[0];
                goal[1] = agent_position[1];
            }

            // Each thread gets its own adaptor & pathfinder → thread-safe
            TileAdaptor adaptor(mapSize, Map);
            Pathfinder pathfinder(adaptor, WEIGHT_PATH);

            auto [path, dist] = pathfinder.search(start[1]*mapSizeX + start[0], goal[1]*mapSizeX + goal[0], mapSize);

            path_many[k] = std::move(path);
            distances_many[k] = dist;
        });
    });

    return {path_many, distances_many};
}


// Multi-threaded version using openMP
inline std::tuple<std::vector<std::vector<int>>, std::vector<float>> FindPathManyMP(
    const std::vector<int> agent_position,
    const std::vector<int> targets_position,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    // release GIL for true parallelism
    pybind11::gil_scoped_release release;

    std::vector<int> start_goal_pair = get_combination(targets_position.size()/2 + 1, 2);
    size_t n_pairs = start_goal_pair.size() / 2;
    std::vector<std::vector<int>> path_many(n_pairs);
    std::vector<float> distances_many(n_pairs);

    // passing the map size, and the map
    Vectori mapSize(mapSizeX, mapSizeY);

    #pragma omp parallel for schedule(dynamic)
    for (size_t k = 0; k < n_pairs; k++) {
        int start_idx = start_goal_pair[2*k];
        int goal_idx  = start_goal_pair[2*k + 1];
        int start[2], goal[2];

        if (start_idx != 0) {
            start[0] = targets_position[2*(start_idx-1)];
            start[1] = targets_position[2*(start_idx-1)+1];
        } else {
            start[0] = agent_position[0];
            start[1] = agent_position[1];
        }

        if (goal_idx != 0) {
            goal[0] = targets_position[2*(goal_idx-1)];
            goal[1] = targets_position[2*(goal_idx-1)+1];
        } else {
            goal[0] = agent_position[0];
            goal[1] = agent_position[1];
        }

        // Each thread gets its own adaptor & pathfinder → thread-safe
        TileAdaptor adaptor(mapSize, Map);
        Pathfinder pathfinder(adaptor, WEIGHT_PATH);

        auto [path, dist] = pathfinder.search(start[1]*mapSizeX + start[0], goal[1]*mapSizeX + goal[0], mapSize);

        path_many[k] = std::move(path);
        distances_many[k] = dist;
    }
    return {path_many, distances_many};
}


inline std::tuple<std::vector<std::vector<int>>, std::vector<float>> FindPathMany(
    const std::vector<int> agent_position,
    const std::vector<int> targets_position,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    std::vector<int> start_goal_pair = get_combination(targets_position.size()/2 + 1, 2);
    std::vector<std::vector<int>> path_many;
    std::vector<float> distances_many;
    int start[2];
    int goal[2];

    //Instantiating our path adaptor
    //passing the map size, and the map
    Vectori mapSize(mapSizeX, mapSizeY);
    TileAdaptor adaptor(mapSize, Map);
    //This is a bit of an exageration here for the weight, but it did make my performance test go from 8s to 2s
    // Pathfinder pathfinder(adaptor, 100.f /*weight*/);
    Pathfinder pathfinder(adaptor, WEIGHT_PATH);

    for (unsigned long idx = 0; idx < start_goal_pair.size(); idx = idx + 2)
    {
        int start_idx = start_goal_pair[idx];
        int goal_idx = start_goal_pair[idx+1];

        if (start_idx != 0)
        {
            start[0] = targets_position[2*(start_idx-1)];
            start[1] = targets_position[2*(start_idx-1)+1];
        }
        else
        {
            start[0] = agent_position[0];
            start[1] = agent_position[1];
        }

        if (goal_idx != 0)
        {
            goal[0] = targets_position[2*(goal_idx-1)];
            goal[1] = targets_position[2*(goal_idx-1)+1];

        }
        else
        {
            goal[0] = agent_position[0];
            goal[1] = agent_position[1];
        }

        //doing the search
        auto [path, distance] = pathfinder.search(start[1]*mapSizeX+start[0], goal[1]*mapSizeX+goal[0], mapSize);
        path_many.push_back(path);
        distances_many.push_back(distance);

        // Regenerate the neighbors for next run
        if (likely(idx < start_goal_pair.size()-1))
        {
            pathfinder.generateNodes();
        }
    }
    return {path_many, distances_many};
}


inline std::tuple<std::vector<int>, float> FindPath(
    const std::vector<int> &startPoint,
    const std::vector<int> &endPoint,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    //Instantiating our path adaptor
    //passing the map size, and the map
    Vectori mapSize(mapSizeX, mapSizeY);

    TileAdaptor adaptor(mapSize, Map);
    
    //This is a bit of an exageration here for the weight, but it did make my performance test go from 8s to 2s
    Pathfinder pathfinder(adaptor, 100.f /*weight*/);

    //The map was edited so we need to regenerate teh neighbors
    // pathfinder.generateNodes();

    //doing the search
    //merly to show the point of how it work
    //as it would have been way easier to simply transform the vector to id and pass it to search
    // auto [Path, Distance] = pathfinder.search(startPoint[1]*mapSizeX+startPoint[0], endPoint[1]*mapSizeX+endPoint[0], mapSize);

    return pathfinder.search(startPoint[1]*mapSizeX+startPoint[0], endPoint[1]*mapSizeX+endPoint[0], mapSize);
}


inline PYBIND11_MODULE(LazyThetaStarPython, module) {
    module.doc() = "Python wrapper of Lazy Theta Star c++ implementation";

    module.def("FindPath", &FindPath, "Find a collision-free path");
    module.def("FindPathMany", &FindPathMany, "Find all the collision-free paths");
    module.def("FindPathManyMP", &FindPathManyMP, "Multi-threading version by openMP: Find all the collision-free paths");
    module.def("FindPathManyTBB", &FindPathManyTBB, "Multi-threading version by Intel TBB: Find all the collision-free paths");
}