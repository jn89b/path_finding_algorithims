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
        self.collision_bubble = 5.0
        
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
            node_position[1] < 0):
            #print("Out of bounds", node_position)
            return False
        
    def main(self):
        step_size = 1
        move  =  [[-step_size, 0 ], # go up
                  [ 0, -step_size], # go left
                  [ step_size, 0 ], # go down
                  [ 0, step_size]] # go right
        self.init_node()
        
        count = 0 
        """main implementation"""
        while not self.openset.empty():
        #while len(self.openset) > 0:
            count = count + 1
            #print(count)
            if count >= 2000:
                print("iterations too much")
                return self.closedset
            
            if self.openset.empty():
                print("No more moves")
            
            #pop node off from priority queue and add into closedset
            cost,current_node = self.openset.get()            
            self.closedset[current_node.position] = current_node
            
            #check if we hit the goal 
            if current_node.position == self.end_node.position:
                print("Goal reached", current_node.position)
                path = return_path(current_node, grid)
                return path
    
            #move generation
            children = []
            for new_position in move:
                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
                
                # Make sure within range (check if within maze boundary)
                if self.is_move_valid(node_position) == False:
                    continue
        
                # Make sure walkable terrain
                if self.grid[node_position] != 0:
                    continue
                
                #check collision bubble here
                dist, obst_index = self.find_closest_obstacle(self.obstacle_list, node_position)
                print("checking", self.obstacle_list[obst_index])
                if self.is_collision(dist):
                    continue
                
                #create new node
                new_node = Node(current_node, node_position)
                
                #put to priority queue
                children.append(new_node)
                
            #check each children 
            for child in children:
                #check if children is already visited
                if child.position in self.closedset:
                    #print("Exists", child.position)
                    continue 
                
                ## Heuristic costs calculated here, this is using eucledian distance
                cost = 1
                child.g = current_node.g + cost
                child.h = compute_euclidean(child.position, self.end_node)
                child.f = child.g + child.h
                
                #add to open set
                #print("putting in", child.position)
                self.openset.put((child.f, child))


#%% general function setup
def generate_grid(grid_row, grid_col):
    grid = []
    for i in range(grid_row):
        grid.append([])
        for j in range(grid_col):
            grid[i].append(0)
    
    return np.array(grid)

def add_obstacles(grid, obstacle_list):
    """"add obstacles to grid location"""
    grid = grid[:]
    for obstacle in obstacle_list:
        (grid[obstacle[0], obstacle[1]]) = 1
        
    return obstacle_list

def compute_euclidean(position, goal):
    heuristic =  (((position[0] - goal.position[0]) ** 2) + 
                       ((position[1] - goal.position[1]) ** 2))        
    
    return heuristic
    
#This function return the path of the search
def return_path(current_node,grid):
    path = []
    grid_row, grid_col = np.shape(grid)
    no_rows = grid_row
    no_columns = grid_col
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
    ax = plt.gca()
    ax.set_xlim([-1, grid_col])
    ax.set_ylim([-1, grid_row])

    for obstacle in obstacles:
       ax.scatter(obstacle[0],obstacle[1], color='red')
       
    #plot waypoints
    x = [x[0] for x in waypoint_list]
    y = [y[1] for y in waypoint_list]
    ax.plot(x,y,'bo', linestyle="-")
    #ax.scatter(goal[0], goal[1], color='purple', marker="+")

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
    grid_row = 100 #y
    grid_col = 100 #x
    grid = generate_grid(grid_row, grid_col)
    
    obstacle_list = [(80,50), (20,40)]
    obstacle_list = add_obstacles(grid, obstacle_list)
    
    landing_zones = [(40,45), (40,60), (60,45), (60,60)]
    plot_landing_zones(landing_zones)
    
    uav_0 = UAV("uav0", [1,1], 1,landing_zones[1])
    uav_1 = UAV("uav1", [30,0], 0, landing_zones[0])
    uav_2 = UAV("uav2", [15,0], 2, landing_zones[2])
    
    uav_list = [uav_0, uav_1, uav_2]
    uav_loc = [uav_0.starting_position, uav_1.starting_position, uav_2.starting_position]
    
    waypoint_dict = {}
    path_obst = []
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
        astar = Astar(grid_copy, new_obstacle,  uav.starting_position, uav.goalpoint)
        uav.path = astar.main()
        waypoint_dict[uav.id] = uav.path
        plot_path(grid_row, grid_col, uav.path, new_obstacle , uav.goalpoint) 
    


    
    
    
    
    
    
        
        
        