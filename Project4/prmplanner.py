# prmplanner.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)

from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
import numpy as np
import math
import random

disk_robot = True #(change this to False for the advanced extension)


obstacles = None # the obstacles
robot_radius = None # the radius of the robot
robot_width = None # the width of the OBB robot (advanced extension)
robot_height = None # the height of the OBB robot (advanced extension)


# ----------------------------------------
# modify the code below
# ----------------------------------------

# Construction phase: Build the roadmap
# You should incrementally sample configurations according to a strategy and add them to the roadmap,
# select the neighbors of each sample according to a distance function and strategy, and
# attempt to connect the sample to its neighbors using a local planner, leading to corresponding edges
# See graph.py to get familiar with the Roadmap class

def build_roadmap(q_range, robot_dim, scene_obstacles):

    global obstacles, robot_width, robot_height, robot_radius

    obstacles = scene_obstacles # setting the global obstacle variable

    x_limit = q_range[0] # the range of x-positions for the robot
    y_limit = q_range[1] # the range of y-positions for the robot
    theta_limit = q_range[2] # the range of orientations for the robot (advanced extension)

    robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box

    robot_radius = max(robot_width, robot_height)/2

    # the roadmap
    graph = Roadmap()
    sample_number = 500
    # graph.addVertex((0, 0))

    # seed(1)

    x = np.linspace(-51, 51, 50)
    y = np.linspace(-51, 51, 35)
    x_samples = []
    y_samples = []
    for i in range(len(x)):
        for j in range(len(y)):
            x_c = x[i] + np.random.uniform(-1.5, 1.5)
            y_c = y[j] + np.random.uniform(-1.5, 1.5)
            C = (x_c, y_c)
            if collision(C, obstacles, robot_radius) == True:
                graph.addVertex(C)

    vertex = Roadmap.getVertices(graph)[0]
    visited = []
    graph.computeConnectedComponents(vertex, obstacles, robot_dim, visited = [])

    # uncomment this to export the roadmap to a file
    graph.saveRoadmap("prm_roadmap.txt")
    return graph

# ----------------------------------------
# modify the code below
# ----------------------------------------

# Query phase: Connect start and goal to roadmap and find a path using A*
# (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# The returned path should be a list of configurations, including the local paths along roadmap edges
# Make sure that start and goal configurations are collision-free. Otherwise return None

def find_path(q_start, q_goal, graph):
    path  = []

    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()

    # Use the PriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)

    # fp = open('prm1.csv', 'r')
    # lines = fp.readlines()
    # query_parameters = lines[2].split(',')
    # fp.close()
    #
    #
    # start = (float(query_parameters[0]), -float(query_parameters[1]))
    # goal = (float(query_parameters[3]), -float(query_parameters[4]))

    print(q_start)
    print(q_goal)

    q_start1 = (q_start[0], q_start[1])
    q_goal1 = (q_goal[0], q_goal[1])


    graph.addVertex(q_start1)
    graph.addVertex(q_goal1)
    parent = [' ' for row in range(len(Roadmap.getVertices(graph)))]
    start_o = Roadmap.getVertices(graph)[-2]
    goal_o = Roadmap.getVertices(graph)[-1]

    start_neighbors, start_neighbor_distances = nearest_neighbors(graph, start_o)
    goal_neighbors, goal_neighbor_distances = nearest_neighbors(graph, goal_o)

    for a, neighbor in enumerate(start_neighbors):
        graph.addEdge(start_o, neighbor, start_neighbor_distances[a])

    for b, neighbor in enumerate(goal_neighbors):
        graph.addEdge(goal_o, neighbor, goal_neighbor_distances[b])

    g = 0
    h = distance(q_start1, q_goal1)
    f = g + h
    open_set.put(q_start1, Value(g=g, f=f))



    while len(open_set) > 0:
        print('while starts')
        next, value = open_set.pop()
        print("popped node",next)
        print('open_set', len(open_set))
        x = next[0]
        y = next[1]

        print(x, y)
        g = value.g
        closed_set.add(next)
        print(len(closed_set))
        if next[0]==q_goal[0] and next[1]==q_goal[1]:
            k=(q_goal[0],q_goal[1])
            while k[0] != q_start1[0] and k[1] != q_start1[1]:
                path.append(k)
                k=parent[graph_object(graph,k).id]
            path.append(q_start1)

            break

        else:
            vertex = graph_object(graph, next)

            edges = RoadmapVertex.getEdges(vertex)
            print
            print(edges, 'edges')
            if edges == None:
                break

            for edge in edges:
                print(edge.id, 'edges')
                for v in graph.vertices:
                    if edge.id == v.id:
                        child = v.q
                        print("child",child)
                        if child not in closed_set:
                            g2 = g + distance(child, next)
                            if child not in open_set or open_set.get(child).g > g2:
                                f2 = g2 + distance(child, q_goal)
                                open_set.put(child, Value(f=f2,g=g2))
                                parent[v.id] = next


    return path


def graph_object(graph, q):
    for v in graph.vertices:
        if v.q == q:
            return v
# ----------------------------------------
# below are some functions that you may want to populate/modify and use above
# ----------------------------------------

def nearest_neighbors(graph, q, max_dist=10.0):
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances
    """
    neighbors = []
    neighbor_distances = []

    for point in Roadmap.getVertices(graph):
        if q.q != point.q:
            d = distance(q.q, point.q)
            if d <= max_dist:
                neighbors.append(point)
                neighbor_distances.append(d)
            else:
                pass

    return neighbors, neighbor_distances


def k_nearest_neighbors(graph, q, K=10):
    """
        Returns the K-nearest roadmap vertices for a given configuration q.
        You may also want to return the corresponding distances
    """

    return None

def distance (q1, q2):
    """
        Returns the distance between two configurations.
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration
    """
    distance = math.sqrt((q1[0] - q2[0])**2 + (q1[1] - q2[1])**2)

    return distance

def collision(q, obstacles, robot_radius):
    """
        Determines whether the robot placed at configuration q will collide with the list of AABB obstacles.
    """
    for obstacle in obstacles:
        # print(obstacle)
        min_x = obstacle.x_min - robot_radius
        max_x = obstacle.x_max + robot_radius
        min_y = obstacle.y_min - robot_radius
        max_y = obstacle.y_max + robot_radius

        if (q[0] > min_x and q[0] < max_x) and (q[1] > min_y and q[1] < max_y):
            return False

    return True


def interpolate (q1, q2, stepsize):
    """
        Returns an interpolated local path between two given configurations.
        It can be used to determine whether an edge between vertices is collision-free.
    """


    m = stepsize
    n = distance(q1, q2) - stepsize
    x = (m*q2[0] + n*q1[0])/distance(q1, q2)
    y = (m*q2[1] + n*q1[1])/distance(q1, q2)


    return (x, y)




if __name__ == "__main__":

    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
