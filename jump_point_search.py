# -*- coding: utf-8 -*-
"""
Created on Thu Oct 28 17:02:07 2021

@author: jn89b
"""

# -*- coding: utf-8 -*-
"""
Created on Wed Oct 20 16:14:14 2021

@author: jn89b

Astar 
take in grid size
take in collision objects 
take in x amount of drones
"""
from queue import PriorityQueue
from scipy import spatial

import numpy as np

import collections
import heapq
import numpy as np 
import matplotlib.pyplot as plt
import math as m

class PreFlightPlanner():
    """Look at unique permutations for zone assignments
    and run order
    if run order creates and """
    def __init__(self):
        pass

class UAV():
    """this is a fake uav has an id and position"""
    def __init__(self, id_name , position, index, goal):
        self.id = id_name
        self.starting_position = position 
        self.goalpoint = goal
        self.zone_index = index
        self.path = None

class Node():
    """
    parent = parent of current node
    posiition = position of node right now it will be x,y coordinates
    g = cost from start to current to node
    h = heuristic 
    f = is total cost
    """
    def __init__(self, parent, position):
        self.parent = parent
        self.position = position
        
        self.g = 0
        self.h = 0
        self.f = 0
        
    def __lt__(self, other):
        return self.f < other.f
    
    # Compare nodes
    def __eq__(self, other):
        return self.position == other.position

    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.position, self.f))



#%% general function setup
def generate_grid(grid_row, grid_col, grid_height):
    grid = []
    grid = np.zeros((grid_height, grid_row, grid_col))
    
    return grid

def add_obstacles(grid, obstacle_list):
    """"add obstacles to grid location"""
    for obstacle in obstacle_list:
        (grid[obstacle[2],obstacle[0], obstacle[1]]) = 1
        
    return obstacle_list

def compute_euclidean(position, goal):

    distance =  (((position[0] - goal.position[0]) ** 2) + 
                       ((position[1] - goal.position[1]) ** 2) +
                       ((position[2] - goal.position[2]) ** 2))
    
    z = 1.0 * position[2]
    heuristic = distance# + z
    
    return heuristic
    
#This function return the path of the search
def return_path(current_node,grid):
    path = []
    no_rows = len(grid)
    no_columns = len(grid)
    # here we create the initialized result maze with -1 in every position
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    current = current_node
    
    while current is not None:
        path.append(current.position)
        current = current.parent
    # Return reversed path as we need to show from start to end path
    path = path[::-1]
    start_value = 0
    # we update the path of start to end found by A-star serch with every step incremented by 1
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1
        
    return path

"""
need to plot:
    uav starting point
    uav goal point
    show the obstacles 
    pathway trajectory
        
"""

def plot_path(grid_row, grid_col,waypoint_list, obstacles, goal):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlim([-1, grid_col])
    ax.set_ylim([-1, grid_row])
    ax.set_zlim([-1, 30])

    for obstacle in obstacles:
       ax.scatter(obstacle[0],obstacle[1], obstacle[2], color='red')
       
    #plot waypoints
    x = [x[0] for x in waypoint_list]
    y = [y[1] for y in waypoint_list]
    z = [z[2] for z in waypoint_list]
    ax.plot3D(x,y,z, 'bo', linestyle="-")
    ax.scatter(goal[0], goal[1], goal[2], color='purple', marker="+")

    plt.grid()
    plt.show()

def plot_landing_zones(landing_zones):
    fig = plt.figure()
    ax = plt.gca()
    ax.set_xlim([-1, grid_col])
    ax.set_ylim([-1, grid_row])
    
    for zone in landing_zones:
       ax.scatter(zone[0],zone[1], color='black')
       
    plt.grid()
    plt.show()
    
def return_other_zones(zones, index):
    """return all other zones not assigned to uav to make as a no fly zone"""
    copy = zones
    copy.pop(index)
    return copy

def return_other_uavs(uavs, uav_index):
    """return all other zones not assigned to uav to make as a no fly zone"""
    copy = uavs
    copy.pop(uav_index)
    return copy

if __name__ == '__main__':
    grid_row = 5
    grid_col = 5
    grid_height = 5
    grid = generate_grid(grid_row, grid_col,grid_height)
    
    
    static_obstacle_list = [(3,3)]
    
    some_list = []
    for static_obstacle in static_obstacle_list:
        x = static_obstacle[0]
        y = static_obstacle[1]
        for z in range(3):
            some_list.append((x,y,z))
    
    obstacle_list = some_list
    obstacle_list = add_obstacles(grid, obstacle_list)
    
    landing_zones = [(30,30,10),(40,60,10), (60,45,10), (60,60,10)]
    
    #uav0
    uav_0 = UAV("uav0", [0,0,10], 0, landing_zones[0])    
    uav_list = [uav_0]
    uav_loc = [uav_0.starting_position]
    path_obst = []
    waypoint_dict={}
    
    #%% Main Method testing for Point Jump Search (PJS)
    
    
    #astar = Astar(grid, obstacle_list,  uav_0.starting_position, uav_0.goalpoint)
    #uav_0.path = astar.main()
    plot_path(grid_row, grid_col,uav_0.path, obstacle_list, uav_0.goalpoint)
    
    """
    for idx, uav in enumerate(uav_list):
        print(idx)
        if idx == 0:
            new_obstacle = obstacle_list + return_other_zones(landing_zones[:], uav.zone_index) + return_other_uavs(uav_loc[:], idx)
        else:
            #append more than path to uav
            path_obst.append(uav_list[idx-1].path)
            flat_list = [item for sublist in path_obst for item in sublist]
            new_obstacle = obstacle_list + return_other_zones(landing_zones[:], uav.zone_index) + return_other_uavs(uav_loc[:], idx) + flat_list
            
        grid_copy = grid.copy()
        new_obstacle = add_obstacles(grid_copy, new_obstacle)
        print("uav")
        astar = Astar(grid_copy, new_obstacle,  uav.starting_position, uav.goalpoint)
        uav.path = astar.main()
        waypoint_dict[uav.id] = uav.path
        plot_path(grid_row, grid_col, uav.path, new_obstacle , uav.goalpoint) 
    """


    
    
    
    
    
    
        
        
        