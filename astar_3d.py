# -*- coding: utf-8 -*-
"""
Created on Thu Oct 21 15:33:46 2021

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


class UAV():
    """this is a fake uav has an id and position"""
    def __init__(self, id_name , position, goal):
        self.id = id_name
        self.starting_position = position
        self.goalpoint = goal
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
    
class Astar():

    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.collision_bubble = 5
        self.height_boundary = 100
        self.ground_boundary = 5

        self.openset = PriorityQueue() # priority queue
        self.closedset = {}


    def is_collision(self,distance):
        """check if there is a collision if so return True"""
        if distance <= 3:
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
        
        self.end_node = Node(None, tuple(self.goal))
        self.end_node.g = self.end_node.h = self.end_node.f = 0
                            
    def main(self):
        
        move  =  [[-1, 0, 0 ], # go up
                  [ 0, -1, 0], # go left
                  [ 1, 0 , 0], # go down
                  [ 0, 1, 0 ],
                  [ 0, 0 , 1], #go up 
                  [ 0, 0 , -1]] # go right
        
        self.init_node()
        
        """main implementation"""
        while not self.openset.empty():
            
            #pop node off from priority queue and add into closedset
            cost,current_node = self.openset.get()
            #closedset = {current_node.position:current_node}
            self.closedset[current_node.position] = current_node
            
            #check if we hit the goal 
            if current_node.position == self.end_node.position:
                print("Goal reached", current_node.position)
                path = return_path(current_node, grid)
                print(path)
                
                return path
    
            #children node generation 
            children = []
            for new_position in move:
                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1], current_node.position[2] + new_position[2])
                print(node_position)
                # Make sure within range (check if within maze boundary)
                if (node_position[0] > (len(self.grid) - 1) or 
                    node_position[0] < 0 or 
                    node_position[1] > (len(self.grid)-1) or 
                    node_position[1] < 0 or
                    node_position[2] > self.height_boundary or
                    node_position[2] < self.ground_boundary ):
                    #print("Out of bounds", node_position)
                    continue
        
                # Make sure walkable 
                if self.grid[node_position] != 0:
                    continue
                
                #check collision bubble here
                dist, obst_index = self.find_closest_obstacle(obstacle_list, node_position)
                if self.is_collision(dist):
                    #print("Collision happens", dist)
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
                child.h = compute_euclidean(child, self.end_node)
                child.f = child.g + child.h
                
                #add to open set
                #print("putting in", child.position)
                self.openset.put((child.f, child))
                

#%% general function setup
def generate_grid(grid_row, grid_col, grid_height):
    grid = []
    for i in range(grid_row):
        grid.append([])
        for j in range(grid_col):
            grid[i].append(0)
    
    return np.array(grid)

def add_obstacles(grid, obstacle_list):
    """"add obstacles to grid location"""
    for obstacle in obstacle_list:
        (grid[obstacle[0], obstacle[1], obstacle[2]]) = 1
        
    return obstacle_list

def compute_euclidean(current, goal):
    heuristic =  (((current.position[0] - goal.position[0]) ** 2) + 
                       ((current.position[1] - goal.position[1]) ** 2) + 
                       ((current.position[2] - goal.position[2]) ** 2))        
    
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
    ax = plt.gca()
    ax.set_xlim([-1, grid_col])
    ax.set_ylim([-1, grid_row])

    for obstacle in obstacles:
       ax.scatter(obstacle[0],obstacle[1], color='red')
       
    #plot waypoints
    x = [x[0] for x in waypoint_list]
    y = [y[1] for y in waypoint_list]
    ax.plot(x,y,'bo', linestyle="-")
    ax.scatter(goal[0], goal[1], color='purple', marker="+")

    plt.grid()
    plt.show()

if __name__ == '__main__':
    grid_row = 50
    grid_col = 75
    grid_height = 100
    grid = generate_grid(grid_row, grid_col)
    
    obstacle_list = [(20,10,10), (25,10,10), (26,10,10)]
    obstacle_list = add_obstacles(grid, obstacle_list)
    
    landing_zones = [(30,20,5), (30,30,5), (20,70,5), (30,70,5)]
    
    #uav0
    uav_0 = UAV("uav0", [0,0,15], landing_zones[0])
    astar = Astar(grid, uav_0.starting_position, uav_0.goalpoint)
    uav_0.path = astar.main()
    plot_path(grid_row, grid_col,uav_0.path, obstacle_list, uav_0.goalpoint)
    
    #uav 1
    new_obstacle = obstacle_list.extend(uav_0.path)
    new_obstacle = add_obstacles(grid, obstacle_list)
    uav_1 = UAV("uav1", [15,0,20], landing_zones[1])
    astar_2 = Astar(grid, uav_1.starting_position, uav_1.goalpoint)
    uav_1.path = astar_2.main()
    
    """plots"""
    plot_path(grid_row, grid_col,uav_1.path, new_obstacle, uav_1.goalpoint)
    
    
    
    
    
    
        
        
        