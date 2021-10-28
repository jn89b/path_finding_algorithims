# -*- coding: utf-8 -*-
"""
Created on Tue Oct 26 12:27:11 2021

@author: jn89b
"""


import collections
import heapq
import numpy as np 
import matplotlib.pyplot as plt
import timeit
import itertools

"""
what is an intersection 
with the dot product 
"""

class Node:
  RED = True
  BLACK = False

  def __init__(self, key, color = RED):
    if not type(color) == bool:
      raise TypeError("Bad value for color parameter, expected True/False but given %s" % color)
    self.color = color
    self.key = key
    self.left = self.right = self.parent = NilNode.instance()

  def __str__(self, level = 0, indent = "   "):
    s = level * indent + str(self.key)
    if self.left:
      s = s + "\n" + self.left.__str__(level + 1, indent)
    if self.right:
      s = s + "\n" + self.right.__str__(level + 1, indent)
    return s

  def __nonzero__(self):
    return True

  def __bool__(self):
    return True


class NilNode(Node):
  __instance__ = None

  @classmethod
  def instance(self):
    if self.__instance__ is None:
      self.__instance__ = NilNode()
    return self.__instance__

  def __init__(self):
    self.color = Node.BLACK
    self.key = None
    self.left = self.right = self.parent = None

  def __nonzero__(self):
    return False

  def __bool__(self):
    return False

class RedBlackTree:
  def __init__(self):
    self.root = NilNode.instance()
    self.size = 0
    
  def __str__(self):
    return ("(root.size = %d)\n" % self.size)  + str(self.root)

  def add(self, key):
    self.insert(Node(key))

  def insert(self, x):
    self.__insert_helper(x)

    x.color = Node.RED
    while x != self.root and x.parent.color == Node.RED:
      if x.parent == x.parent.parent.left:
        y = x.parent.parent.right
        if y and y.color == Node.RED:
          x.parent.color = Node.BLACK
          y.color = Node.BLACK
          x.parent.parent.color = Node.RED
          x = x.parent.parent
        else:
          if x == x.parent.right:
            x = x.parent
            self.__left_rotate(x)
          x.parent.color = Node.BLACK
          x.parent.parent.color = Node.RED
          self.__right_rotate(x.parent.parent)
      else:
        y = x.parent.parent.left
        if y and y.color == Node.RED:
          x.parent.color = Node.BLACK
          y.color = Node.BLACK
          x.parent.parent.color = Node.RED
          x = x.parent.parent
        else:
          if x == x.parent.left:
            x = x.parent
            self.__right_rotate(x)
          x.parent.color = Node.BLACK
          x.parent.parent.color = Node.RED
          self.__left_rotate(x.parent.parent)
    self.root.color = Node.BLACK

  def delete(self, z):
    if not z.left or not z.right:
      y = z
    else:
      y = self.successor(z)
    if not y.left:
      x = y.right
    else:
      x = y.left
    x.parent = y.parent

    if not y.parent:
      self.root = x
    else:
      if y == y.parent.left:
        y.parent.left = x
      else:
        y.parent.right = x

    if y != z: z.key = y.key

    if y.color == Node.BLACK:
      self.__delete_fixup(x)

    self.size -= 1
    return y

  def minimum(self, x = None):
    if x is None: x = self.root
    while x.left:
      x = x.left
    return x

  def maximum(self, x = None):
    if x is None: x = self.root
    while x.right:
      x = x.right
    return x

  def successor(self, x):
    if x.right:
      return self.minimum(x.right)
    y = x.parent
    while y and x == y.right:
      x = y
      y = y.parent
    return y

  def predecessor(self, x):
    if x.left:
      return self.maximum(x.left)
    y = x.parent
    while y and x == y.left:
      x = y
      y = y.parent
    return y

  def inorder_walk(self, x = None):
    if x is None: x = self.root
    x = self.minimum()
    while x:
      yield x.key
      x = self.successor(x)

  def reverse_inorder_walk(self, x = None):
    if x is None: x = self.root
    x = self.maximum()
    while x:
      yield x.key
      x = self.predecessor(x)

  def search(self, key, x = None):
    if x is None: x = self.root
    while x and x.key != key:
      if key < x.key:
        x = x.left
      else:
        x = x.right
    return x

  def is_empty(self):
    return bool(self.root)

  def black_height(self, x = None):
    if x is None: x = self.root
    height = 0
    while x:
      x = x.left
      if not x or x.is_black():
        height += 1
    return height

  def __left_rotate(self, x):
    if not x.right:
      raise "x.right is nil!"
    y = x.right
    x.right = y.left
    if y.left: y.left.parent = x
    y.parent = x.parent
    if not x.parent:
      self.root = y
    else:
      if x == x.parent.left:
        x.parent.left = y
      else:
        x.parent.right = y
    y.left = x
    x.parent = y

  def __right_rotate(self, x):
    if not x.left:
      raise "x.left is nil!"
    y = x.left
    x.left = y.right
    if y.right: y.right.parent = x
    y.parent = x.parent
    if not x.parent:
      self.root = y
    else:
      if x == x.parent.left:
        x.parent.left = y
      else:
        x.parent.right = y
    y.right = x
    x.parent = y

  def __insert_helper(self, z):
    y = NilNode.instance()
    x = self.root
    while x:
      y = x
      if z.key < x.key:
        x = x.left
      else:
        x = x.right
    
    z.parent = y
    if not y:
      self.root = z
    else:
      if z.key < y.key:
        y.left = z
      else:
        y.right = z
    
    self.size += 1

  def __delete_fixup(self, x):
    while x != self.root and x.color == Node.BLACK:
      if x == x.parent.left:
        w = x.parent.right
        if w.color == Node.RED:
          w.color = Node.BLACK
          x.parent.color = Node.RED
          self.__left_rotate(x.parent)
          w = x.parent.right
        if w.left.color == Node.BLACK and w.right.color == Node.BLACK:
          w.color = Node.RED
          x = x.parent
        else:
          if w.right.color == Node.BLACK:
            w.left.color = Node.BLACK
            w.color = Node.RED
            self.__right_rotate(w)
            w = x.parent.right
          w.color = x.parent.color
          x.parent.color = Node.BLACK
          w.right.color = Node.BLACK
          self.__left_rotate(x.parent)
          x = self.root
      else:
        w = x.parent.left
        if w.color == Node.RED:
          w.color = Node.BLACK
          x.parent.color = Node.RED
          self.__right_rotate(x.parent)
          w = x.parent.left
        if w.right.color == Node.BLACK and w.left.color == Node.BLACK:
          w.color = Node.RED
          x = x.parent
        else:
          if w.left.color == Node.BLACK:
            w.right.color = Node.BLACK
            w.color = Node.RED
            self.__left_rotate(w)
            w = x.parent.left
          w.color = x.parent.color
          x.parent.color = Node.BLACK
          w.left.color = Node.BLACK
          self.__right_rotate(x.parent)
          x = self.root
    x.color = Node.BLACK
    
  

class Point:
    def __init__(self, x, y, ptype = 0):
        self.x = x
        self.y = y
        self.ptype = ptype # 0 means left, 1 means right
        self.other_end = None

    def subtract(self, p):
        return Point(self.x - p.x, self.y - p.y)

    def __str__(self):
        return '(' + str(self.x) + ', ' + str(self.y) + ')'
    
def compare(p1, p2):
    if p1.x < p2.x:
        return -1
    elif p1.x > p2.x:
        return 1
  
    if p1.x == p2.x:
        if p1.ptype == p2.ptype:
            if p1.y < p2.y:
                return -1
            else:
                return 1
        else:
            if p1.ptype == 0:
                return -1
            else:
                return 1
                
def any_segments_intersect(S):
    # Step 1
    T = RedBlackTree()

    # Step 2
    sortedS = sorted(S, cmp=lambda x,y: compare(x,y))
    sortedS = map(lambda x: Node(x), sortedS)
    
    # Step 3
    for point in sortedS:
        if point.key.ptype == 0:
            T.insert(point)
            prd = T.predecessor(point)
            if prd and intersect(point.key, point.key.other_end, prd.key, prd.key.other_end):
                return True
            ssc = T.successor(point)
            if ssc and intersect(point.key, point.key.other_end, ssc.key, ssc.key.other_end):
                return True
        if point.key.ptype == 1:
            prd = T.predecessor(point)
            ssc = T.successor(point)

            if prd and ssc:
                if intersect(prd.key, prd.key.other_end, ssc.key, ssc.key.other_end):
                    return True
            T.delete(point)
    return False

def compute_unit_vector(position, goal):

    position = np.array(position)
    goal = np.array(goal)   

    magnitude = np.linalg.norm(position-goal)    
    print("mag", magnitude)
    vector = [goal[0] - position[0], goal[1] - position[1]]
    unit_vector = [vector[0]/magnitude, vector[1]/magnitude] 
    
    return vector

def get_vector(position,goal):
    pass

class UAV():
    """this is a fake uav has an id and position"""
    def __init__(self, id_name , position, zone_index, goal):
        self.id = id_name
        self.starting_position = position
        self.goalpoint = goal
        self.zone_index = zone_index
        self.path = None


class PreFlightPlanner():
    """
    Look at unique permutations of run order and assignments
    given a list of 2 uavs and 4 landing zones 
    draw line of intersection between 
    """
    def __init__(self, landing_zones):
        self.landing_zones = landing_zones
                
    
    def generate_drone_queue(self):
        """generate a queue of drones who are requesting the service return a list 
        of uavs based on a specific priority"""
    
    def generate_landing_permutations(self, uav_list, zone_list):
        """permutate a unique list of uav order with landing zones"""        
        all_combinations = []
        
        #uav_loc_list = [uav.id for uav in uav_list]
        list1_permutations = itertools.permutations(uav_list, len(self.landing_zones))
        print(list1_permutations)
        
        for each_permutation in list1_permutations:
            zipped = zip(each_permutation, zone_list)
            all_combinations.append(list(zipped))
            
        return all_combinations
    
    def generate_landing_orders(self, landing_combinations):
        """create a list of all possible orders of landing from the landing permutations list"""
        landing_order_dict = {}
        count = 0 
        
        for i, each_permutation in enumerate(landing_combinations): 
            for permutation in (each_permutation):    
                combination = list(itertools.permutations(each_permutation))
                landing_order_dict[count] = combination
                count +=1
                
        return landing_order_dict

                
        
if __name__ == '__main__':
    
    landing_zones = [(40,45), (40,60)]#, (60,45), (60,60)]
    
    uav_0 = UAV("uav0", [0,0], None, None)
    uav_1 = UAV("uav1", [0,5], None, None)
    
    uavs = [uav_0, uav_1]
    
    preFlightPlanner = PreFlightPlanner(landing_zones)
    landing_combinations = preFlightPlanner.generate_landing_permutations(uavs, landing_zones)
    all_landing_configs = preFlightPlanner.generate_landing_orders(landing_combinations)
    
    """
    for each set of permutations we want to compute find the vector
    then check if the vectors cross 
    if it crosses we throw out the solution
    
    """
    test = all_landing_configs[0]
    for idx in all_landing_configs:
        configuration = all_landing_configs[idx]
        for uav_list in configuration:
            dist_list = []
            for i, vals in enumerate(uav_list):
                """i want to compute each uavs vector and see if they intersect, if so we continue otherwise solution is found"""
                uav = vals[0]
                uav_goalpoint = vals[1]
                uav.goalpoint = uav_goalpoint
                #print(uav.starting_position)
                """compute magnitude/vector and check if vector crosses and append"""
                dist = compute_unit_vector(uav.starting_position, uav.goalpoint)
                dist_list.append(dist)
                
                if i == len(uav_list)-1:
                    """check for solution"""
                    print("dist", dist_list) 
                
            

            

    
    

    
