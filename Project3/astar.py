# astar.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#


# Compute the optimal path from start to goal.
# The car is moving on a 2D grid and
# its orientation can be chosen from four different directions:
import numpy as np
forward = [[-1,  0], # 0: go north
           [ 0, -1], # 1: go west
           [ 1,  0], # 2: go south
           [ 0,  1]] # 3: go east
#east=forward[3]
#print(east[1])
# The car can perform 3 actions: -1: right turn and then move forward, 0: move forward, 1: left turn and then move forward
action = [-1, 0, 1]
action_name = ['R', 'F', 'L']
cost = [1, 1, 1] # corresponding cost values

# GRID:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = (4, 3, 0) # (grid row, grid col, orientation)
                
goal = (2, 0, 1) # (grid row, grid col, orientation)


heuristic = [[2, 3, 4, 5, 6, 7], # Manhattan distance
        [1, 2, 3, 4, 5, 6],
        [0, 1, 2, 3, 4, 5],
        [1, 2, 3, 4, 5, 6],
        [2, 3, 4, 5, 6, 7]]

from utils import (Value, OrderedSet, PriorityQueue)

"""
Two data structures are provided for your open and closed lists: 

 1. OrderedSet is an ordered collection of unique elements.
 2. PriorityQueue is a key-value container whose `pop()` method always pops out
    the element whose value has the highest priority.

 Common operations of OrderedSet, and PriorityQueue
   len(s): number of elements in the container s
   x in s: test x for membership in s
   x not in s: text x for non-membership in s
   s.clear(): clear s
   s.remove(x): remove the element x from the set s;
                nothing will be done if x is not in s

 Unique operations of OrderedSet:
   s.add(x): add the element x into the set s
   s.pop(): return and remove the LAST added element in s;

 Example:
   s = Set()
   s.add((0,1,2))    # add a triplet into the set
   s.remove((0,1,2)) # remove the element (0,1,2) from the set
   x = s.pop()

 Unique operations of PriorityQueue:
   PriorityQueue(order="min", f=lambda v: v): build up a priority queue
       using the function f to compute the priority based on the value
       of an element
   s.put(x, v): add the element x with value v into the queue
                update the value of x if x is already in the queue
   s.get(x): get the value of the element x
            raise KeyError if x is not in s
   s.pop(): return and remove the element with highest priority in s;
            raise IndexError if s is empty
            if order is "min", the element with minimum f(v) will be popped;
            if order is "max", the element with maximum f(v) will be popped.
 Example:
   s = PriorityQueue(order="min", f=lambda v: v.f)
   s.put((1,1,1), Value(f=2,g=1))
   s.put((2,2,2), Value(f=5,g=2))
   x, v = s.pop()  # the element with minimum value of v.f will be popped
"""

# ----------------------------------------
# modify the code below
# ----------------------------------------
def compute_path(grid,start,goal,cost,heuristic):
   
    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    
    # Use thePriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)      

    # Keep track of the parent of each node. Since the car can take 4 distinct orientations, 
    # for each orientation we can store a 2D array indicating the grid cells. 
    # E.g. parent[0][2][3] will denote the parent when the car is at (2,3) facing up    
    parent = [[[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]]

    # The path of the car
    path =[['-' for row in range(len(grid[0]))] for col in range(len(grid))]
    print(path[4][3])
    x = start[0]
    y = start[1]
    theta = start[2]
    h = heuristic[x][y]
    g = 0
    f = g+h
    open_set.put(start, Value(f=f,g=g))


    child = [0,0,0]
    print("Length of original open_set:",len(open_set))
    parent1=list(start)
    parent_val=1
    print(parent1)
    open_set.pop()
    closed_set.add(parent1)
    #print(parent)
    j=0

    while tuple(parent1)!=goal :

        j=0
        for i in forward:
            print("entered forward loop")
            #print(child)
            f=0
            child[0] = i[0] + parent1[0]
            child[1] = i[1] + parent1[1]

            a = child[0]
            b = child[1]
            print("x and y")
            print(a,b)

            if (a <= 4 and a>=0 and b <= 5 and b>=0):

                print("within limits")
                print(grid[a][b])
                print(grid[a][b] == 0)
                if (grid[a][b] == 0):
                    print("possible space")

                    #orientation
                    if j == 0:
                        child[2] = 0 #for forward
                    elif j == 1:
                        child[2] = 1 #for left
                    elif j==2:
                        child[2] = 2
                    else:
                        child[2] = 3#for right
                    if child[2] - parent1[2] == 0:
                        g2 = cost[1]
                    elif child[2] - parent1[2] == 1:
                        g2 = cost[2]
                    else:
                        g2 = cost[0]
                    g1 = parent_val + 1
                    g_c = g1 + g2
                    h = heuristic[a][b]
                    f_c = g_c + h

                    print("Possible child")
                    print(tuple(child))
                    print("length of open_set",len(open_set))
                    if(tuple(child) not in closed_set and tuple(child) not in open_set):
                        if abs(child[2]-parent1[2])!=2:


                            print("f_c,g_c,h",f_c,g_c,h)
                            print("inside if loop")
                            print("sending it into open list")
                            open_set.put(tuple(child), Value(f=f_c, g=g2))
                            print("length of open_set", len(open_set))
                        elif tuple(child) in open_set:
                            v = open_set.get(tuple(child))
                            if v.f>f_c:
                                v.f=f_c
                                open_set.remove(tuple(child))
                                open_set.put(tuple(child), Value(f=v.f, g=g2))

            print("end of one forward")
            print("updating action")
            j += 1
        print(parent1)
        print("length of open_set",len(open_set))
        print("popping")

        parent2, value = open_set.pop()

        parent_val = value.g
        print("value of popped parent", value.f)
        print("length of open_set after popping",len(open_set))
        print("new parent", parent2)
        parent1 = list(parent2)
        closed_set.add(parent2)
        print("length of closed_set",len(closed_set))
    nodes2=[]
    for nodes1 in closed_set:
        nodes2.append(nodes1)
    for p in range(len(nodes2)-1):
        r1 = list(nodes2[p]);
        r2 = list(nodes2[p+1])
        p1 = r1[0];
        p2 = r1[1]
        if r2[2]-r1[2]==0:
            path[p1][p2] = 'F'
        elif abs(r2[2]-r1[2])==1:
            path[p1][p2] = 'L'
        elif abs(r2[2]-r1[2])==2:
            path[p1][p2] = 'D'
        else:
            path[p1][p2] = 'R'







        # print(child1)

    print('end of one parent')

    print(len(closed_set))
    print(closed_set._container)
    xf = goal[0]
    yf = goal[1]
    path[xf][yf] = '*'

    #print(child1)


    print('end of one parent')

    print(len(closed_set))
    print(closed_set._container)








    # your code: implement A*

    # Initially you may want to ignore theta, that is, plan in 2D.
    # To do so, set actions=forward, cost = [1, 1, 1, 1], and action_name = ['U', 'L', 'R', 'D']
    # Similarly, set parent=[[' ' for row in range(len(grid[0]))] for col in range(len(grid))]

    return path, closed_set


if __name__ == "__main__":
    path,closed=compute_path(grid, init, goal, cost, heuristic)

    for i in range(len(path)):
        print(path[i])

    print("\nExpanded Nodes")
    for node in closed:
        print(node)

"""
To test the correctness of your A* implementation, when using cost = [1, 1, 10] your code should return 

['-', '-', '-', 'R', 'F', 'R']
['-', '-', '-', 'F', '-', 'F']
['*', 'F', 'F', 'F', 'F', 'R']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-'] 

In this case, the elements in your closed set (i.e. the expanded nodes) are: 
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 5, 3)
(0, 3, 0)
(0, 4, 3)
(0, 5, 3)
(1, 5, 2)
(2, 5, 2)
(2, 4, 1)
(2, 3, 1)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

"""