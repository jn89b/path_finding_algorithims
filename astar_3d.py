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
    
class Astar():

    def __init__(self, grid, obs_list,start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        print("goal is")
        self.collision_bubble = 1.0
        self.height_boundary = 5
        self.ground_boundary = 1.0
        
        self.obstacle_list = obs_list

        self.openset = PriorityQueue() # priority queue
        self.closedset = {}
        #self.openset = []

    def is_collision(self,distance):
        """check if there is a collision if so return True"""
        if distance <= self.collision_bubble:
            return True
    
    def find_closest_obstacle(self, obstacles, current_position):
        """find closest obstacle from obstacle list, wrt current position"""
        tree = spatial.KDTree(obstacles)
        dist, obst_index = tree.query(current_position)   
        
        return dist, obst_index
    
    def init_node(self):
        start_node = Node(None,tuple(self.start))
        start_node.g = start_node.h = start_node.f = 0
        self.openset.put((start_node.f, start_node))
        #self.openset.append(start_node)
        self.end_node = Node(None, tuple(self.goal))
        self.end_node.g = self.end_node.h = self.end_node.f = 0

    def is_move_valid(self, node_position):
        """check if move made is valid if so then return True"""
        if (node_position[0] > (len(self.grid) - 1) or 
            node_position[0] < 0 or 
            node_position[1] > (len(self.grid)-1) or 
            node_position[1] < 0 or
            node_position[2] > self.height_boundary or
            node_position[2] < self.ground_boundary ):
            return False
        
    def main(self):
        ss = 1
        
        move  =  [[-ss, 0, 0 ], # go forward
                  [ 0, -ss, 0], # go left
                  [ ss, 0 , 0], # go backward
                  [ 0, ss, 0 ], #go right
                  [ 0, 0 , ss], #go up z  
                  [ 0, 0 , -ss]] # go down z
        
        self.init_node()
        count = 0 
        """main implementation"""
        while not self.openset.empty():
        #while len(self.openset) > 0:
            count = count + 1
            #print(count)
            if count >= 10000:
                print("iterations too much")
                return self.closedset
            
            if self.openset.empty():
                print("No more moves")
            
            #pop node off from priority queue and add into closedset
            cost,current_node = self.openset.get()            
            self.closedset[current_node.position] = current_node
            
            #check if we hit the goal 
            if current_node.position == self.end_node.position:
                #print("Goal reached", current_node.position)
                path = return_path(current_node, grid)
                print("count", count)
                return path
            
            #move generation
            children = []
            for new_position in move:
                # Get node position
                #print(current_node.position)
                
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1],  current_node.position[2] + new_position[2])
                print(node_position)
                # Make sure within range (check if within maze boundary)
                if self.is_move_valid(node_position) == False:
                    #print("move is invalid")
                    continue
        
                # Make sure walkable terrain
                if self.grid[node_position] != 0:
                    #print("not walkable")
                    continue
                
                #check collision bubble here
                dist, obst_index = self.find_closest_obstacle(self.obstacle_list, node_position)
                #print("checking", self.obstacle_list[obst_index])
                if self.is_collision(dist):
                    #print("collision")
                    continue
                
                #create new node
                new_node = Node(current_node, node_position)
                
                # put to possible paths
                children.append(new_node)
                
                #penalty cost if we go down or up
                if ((new_position == move[4]) or (new_position == move[5])):
                    penalty = 1.25
                    #print("penalty")
                else:
                    penalty = 0
                    
            #check each children 
            for child in children:
                #check if children is already visited
                if child.position in self.closedset:
                    #print("Exists", child.position)
                    continue 
                
                ## Heuristic costs calculated here, this is using eucledian distance
                cost = 1
                child.g = current_node.g + cost
                #print("child.position", child.position)
                child.h = compute_euclidean(child.position, self.end_node)
                #print("heuristic", child.h)
                child.f = child.g + (child.h *penalty)
                
                #add to open set
                #print("putting in", child.position)
                self.openset.put((child.f, child))


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
                        ((position[2] - goal.position[2]) ** 2))**(1/2)
    
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
    grid_row = 100
    grid_col = 100
    grid_height = 25
    grid = generate_grid(grid_row, grid_col,grid_height)
    
    
    static_obstacle_list = [(5,5)]
    
    some_list = []
    for static_obstacle in static_obstacle_list:
        x = static_obstacle[0]
        y = static_obstacle[1]
        for z in range(5):
            some_list.append((x,y,z))
    
    obstacle_list = some_list
    obstacle_list = add_obstacles(grid, obstacle_list)
    
    landing_zones = [(10,2,5),(40,60,5), (60,45,10), (60,60,10)]
    
    #uav0
    uav_0 = UAV("uav0", [0,0,5], 1, landing_zones[1])    
    
    #uav 1
    #uav_1 = UAV("uav1", [15,0,15], 1, landing_zones[1])
    
    #uav_list = [uav_0, uav_1]
    #uav_loc = [uav_0.starting_position, uav_1.starting_position]
    uav_list = [uav_0]
    uav_loc = [uav_0.starting_position]
    path_obst = []
    waypoint_dict={}
    
    astar = Astar(grid, obstacle_list,  uav_0.starting_position, uav_0.goalpoint)
    uav_0.path = astar.main()
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


    
    
    
    
    
    
        
        
        