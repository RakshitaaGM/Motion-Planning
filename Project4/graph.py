# graph.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
#
# Authors: Ioannis Karamouzas (ioannis@g.clemson.edu)
#
# import sys
# sys.setrecursionlimit(10**6)

class RoadmapVertex:
    """A class for storing the vertices of your roadmap """

    def __init__(self, q, id):
        self.q = q # the configuration of the roadmap vertex
        self.edges = [] # the neighboring roadmap vertices
        self.id = id # the id of the vertex
        self.connetedComponentNr = -1 #The id of the component to which the vertex belongs

    def getConfiguration(self):
        return self.q

    def getId(self):
        return self.id

    def getEdges(self): # Returns all edges outgoing from the current vertex
        return self.edges

    def getEdge(self, v_id): # Determines whether an outgoing edge exists between the current vertex and a given vertex
        for e in self.edges:
            if e.getId()==v_id: return e
        return None

    def addEdge(self, v_id, dist, path=[]): # Adds an outgoing roadmap edge from the current vertex to vertex with id v_id
        if self.getEdge(v_id): return False
        self.edges.append(RoadmapEdge(self.id, v_id, dist, path))
        return True

    def removeEdge(self, v_id): # Removes the edge from the current vertex to a given vertex with id v_id
        for e in self.edges:
            if e.getId()==v_id:
                self.edges.remove(e)
                return True
        return False

    def getConnectedNr(self):
        return self.connetedComponentNr

    def __hash__(self):
        return hash(self.id)

    def __eq__(self, other):
        return self.id == other.id


class RoadmapEdge:
    """ The class defines an edge along the roadmap """

    def __init__(self, src_id, dest_id, dist, path=[]):
        self.id = dest_id # the id of the edge that denotes the target vertex id
        self.dist = dist # the distance associated with the edge
        self.path = path # the local path
        self.src_id = src_id  # the src_id and dest_id are for debugging purposes, all you need is just the id of the edge
        self.dest_id = dest_id

    def getId(self):
        return self.id

    def getDist(self):
        return self.dist

    def getLocalPath(self):
        return self.path

    def getSource(self):
        return self.src_id

    def getDestination(self):
        return self.dest_id

    def setDist(self, dist):
        self.dist = dist
        return self

    def addPathNode(self, q):
        self.path.append(q)

class Roadmap:
    """ The class defining the roadmap """
    def __init__(self, directed = False):
        self.vertices = [] # the vertices of the graph
        self.directed = directed # whether the graph is directed or not

    def getVertices(self): # returns the vertices as a list
        return self.vertices

    def getNrVertices(self):
        return len(self.vertices)

    def addVertex(self, q):  # adds a configuration to the roadmap
        v = RoadmapVertex(q, len(self.vertices))
        self.vertices.append(v)
        return self.vertices[-1]

    def addEdge(self, u, v, dist, path = []): # adds an edge between two vertices, if such vertices exist
        u_id = u.id
        v_id = v.id
        if u_id!=v_id and 0<=u_id<len(self.vertices) and 0<=v_id<len(self.vertices):
            if (self.directed):
                return self.vertices[u_id].addEdge(v_id, dist, path)
            elif self.vertices[u_id].addEdge(v_id, dist, path):
                return self.vertices[v_id].addEdge(u_id, dist, list(reversed(path)))
        return False

    def removeEdge(self, u, v): # removes the edge between two given vertices
        u_id = u.id
        v_id = v.id
        if u_id!=v_id and 0<=u_id<len(self.vertices) and 0<=v_id<len(self.vertices):
            if (self.directed):
                return self.vertices[u_id].removeEdge(v_id)
            elif self.vertices[u_id].removeEdge(v_id):
                return self.vertices[v_id].removeEdge(u_id)
        return False

    def removeVertex(self, u_id):
        if 0<=u_id<len(self.vertices):
        #for u in self.vertices:
         #  if u.getId() != u_id: continue
            u = self.vertices[u_id]
            for e in u.getEdges():
                if not u.removeEdge(e.getId()): return False
                if not self.directed:
                    if not self.vertices[e.getId()].removeEdge(u_id): return False
            self.vertices.remove(u)
            return True
        return False

    def computeConnectedComponents(self, u, scene_obstacles, robot_dim, visited = []): # implement this to compute the connected components of the undirected graph
        from prmplanner import distance, nearest_neighbors, collision, interpolate
        from obstacles import BoxObstacle
        global obstacles
        from utils import PriorityQueue, Value
        obstacles = scene_obstacles
        robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box
        robot_radius = max(robot_width, robot_height)/2
        queue = {}
        g = 0 # cost between subsequent nodes
        queue[g] = u
        visited.append(u.q)
        while len(queue) != 0:

            g_values = list(sorted(queue.keys()))

            s = queue.pop(g_values[0])

            neighbors, neighbor_distances = nearest_neighbors(self, s)

            if neighbors != None:

                for i, neighbor in enumerate(neighbors):

                    if neighbor.q not in visited:
                        bool = 0
                        for j in range(1, 10):
                            dot = interpolate(s.q, neighbor.q, j * distance(s.q, neighbor.q)/10)
                            if collision(dot, obstacles, robot_radius) == False:
                                bool += 1
                        if bool == 0:
                            self.addEdge(s, neighbor, neighbor_distances[i])
                            g = g_values[0] + distance(s.q, neighbor.q)
                            visited.append(neighbor.q)
                            queue.update({g: neighbor})



    def saveRoadmap(self, filename):
        file = open(filename,"w")
        #export the number of vertices
        file.write(str(self.getNrVertices()) + "\n")
        for v in self.vertices:
            file.write(str(v.id) + "," + ','.join([str(c) for c in v.q]) + "," + str(len(v.edges)) + "\n")
            for e in v.edges:
                file.write(str(e.id) + "," + str(e.dist) + "," + str(len(e.path)))
                if e.path:
                    file.write("\n")
                    for p in e.path:
                        file.write(','.join([str(q) for q in p]) + "\n")
                else:
                     file.write("\n")

        file.close()
