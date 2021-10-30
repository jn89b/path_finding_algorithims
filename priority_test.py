# -*- coding: utf-8 -*-
"""
Created on Thu Oct 28 22:01:14 2021

@author: jn89b
"""

import heapq

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

from queue import PriorityQueue
import heapq

class UniquePriorityQueue(PriorityQueue):
    def _init(self, maxsize):
#        print 'init'
        PriorityQueue._init(self, maxsize)
        self.values = set()

    def _put(self, item, heappush=heapq.heappush):
#        print 'put',item
        if item[1] not in self.values:
            print('uniq',item[1])
            self.values.add(item[1])
            PriorityQueue._put(self, item, heappush)
        else:
            print('dupe',item[1])

    def _get(self, heappop=heapq.heappop):
#        print 'get'
        item = PriorityQueue._get(self, heappop)
#        print 'got',item
        self.values.remove(item[1])
        return item

if __name__=='__main__':
    u = UniquePriorityQueue()

    u.put((0.2, 'foo'))
    u.put((0.3, 'bar'))
    u.put((0.1, 'baz'))
    u.put((0.4, 'foo'))

    while not u.empty():
        item = u.get_nowait()
        print(item)
    