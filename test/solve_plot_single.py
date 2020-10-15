#!/usr/bin/env python3
import os
import sys
sys.path.append(os.getcwd()+'/build')
sys.path.append(os.getcwd()+'/src')
import LazyThetaStarPython
import numpy as np
import time
import matplotlib.pyplot as plt

import Simulator as helper


if __name__ == "__main__":
    # define the world
    map_width_meter = 20.0
    map_width_meter = 20.0
    map_resolution = 2
    value_non_obs = 0 # the cell is empty
    value_obs = 255 # the cell is blocked
    # create a simulator
    Simulator = helper.Simulator(map_width_meter, map_width_meter, map_resolution, value_non_obs, value_obs)
    # number of obstacles
    num_obs = 100
    # generate random obstacles
    Simulator.generate_random_obs(num_obs)
    # convert 2D numpy array to 1D list
    world_map = Simulator.map_array.flatten().tolist()


    # define the start and goal
    start = [0, 0]
    end = [25, 34]
    # solve it
    t0 = time.time()
    path_short = LazyThetaStarPython.FindPath(start, end, world_map, Simulator.map_width, Simulator.map_height)
    t1 = time.time()
    print("Time used for a single path is [sec]: " + str(t1-t0))
    print("This is the path.")
    for idx in range(0,len(path_short),2):
        str_print = str(path_short[idx]) + ', ' + str(path_short[idx+1])
        print(str_print)
    # visualization (uncomment next line if you want to visualize a single path)
    Simulator.plot_single_path(path_short)
