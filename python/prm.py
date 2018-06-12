import numpy as np
import math

class Point:
    x = 0.0
    y = 0.0

    def __init__(self, x, y):
        self.x = x
        self.y = y

class Node:
    node_id = None
    point = None
    

    def __init__(self, x, y, node_id):
        self.point = Point(x,y)
        self.node_id = node_id
        self.neighbors = []

class PRM:

    nodes = []

    def __init__(self, x_max, x_min, y_max, y_min, numNodes):
      self.x_max = x_max
      self.y_max = y_max
      self.x_min = x_min
      self.y_min = y_min
      self.numNodes = numNodes

      self.nodes.append(Node(0,0,0))
      self.nodes.append(Node(18,18,1))
   
    def generateRandomPoints(self,obsVec):
        total = 0
        while(total < self.numNodes):
            p = Node(np.random.choice(self.x_max-self.x_min) + self.x_min,
                     np.random.choice(self.y_max-self.y_min) + self.y_min,
                     total+2)

            if (not self.intersectsObs(p.point, p.point, obsVec) and self.isWithinWorld(p.point)):
                self.nodes.append(p)
                total+=1

    def computeNeighborGraph(self, obsVec):
        for i in self.nodes:
            distanceMap = []
            for j in self.nodes:
                if (i.node_id != j.node_id and not self.intersectsObs(i.point, j.point, obsVec)):
                    distanceMap.append((self.getEuclideanDistance(i.point,j.point),j))

            distanceMap = sorted(distanceMap, key=lambda x: x[0])
            count = 0
            for pair in distanceMap:
                if (count >=10):
                    break
                i.neighbors.append(pair[1]);
                count+=1

    def solveShortestPath(self):
        dist = [10000.0]*(self.numNodes+2)
        vset = [True]*(self.numNodes+2)
        prev = [-1]*(self.numNodes+2)
     
        dist[0] = 0

        while True:
            if (sum(vset) == 0):
                break

            low = 10000.0
            u = -1
            for i in xrange(self.numNodes+2):
                if (vset[i]):
                    if (u == -1 or dist[i] < low):
                        low = dist[i]
                        u = i

            vset[u] = False;

            for v in self.getById(u).neighbors:

                alt = dist[u] + self.getEuclideanDistance(self.getById(u).point, v.point)

                if (alt < dist[v.node_id]):
                    dist[v.node_id] =  alt
                    prev[v.node_id] = u

        node = 1
        path = []
        while True:
            if (node == -1):
                break
            path.append(node)
            node = prev[node]
            
        print path
        return path
            
    def getEuclideanDistance(self,p1, p2):
        return math.sqrt(math.pow((p1.x - p2.x), 2) + math.pow((p1.y - p2.y), 2))

    def isWithinWorld(self,p):
        return (p.x > self.x_min and p.x < self.x_max and p.y > self.y_min and p.y < self.y_max)

    def intersectsObs(self,p1, p2, obsVec):

        x1 = p1.x
        y1 = p1.y
        x2 = p2.x
        y2 = p2.y

        for i in range(len(obsVec)):
            obs = obsVec[i]

            obs_xl = (obs.pose.position.x - obs.scale.x / 2) - 0.5
            obs_xr = (obs.pose.position.x + obs.scale.x / 2) + 0.5
            obs_yb = (obs.pose.position.y - obs.scale.y / 2) - 0.5
            obs_yt = (obs.pose.position.y + obs.scale.y / 2) + 0.5

            #check for the bottom intersection
            bottom = self.lineIntersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xr, obs_yb)
            #left intersect
            left = self.lineIntersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xl, obs_yt)
            #right intersect
            right = self.lineIntersect(x1, y1, x2, y2, obs_xr, obs_yb, obs_xr, obs_yt)
            #top intersect
            top = self.lineIntersect(x1, y1, x2, y2, obs_xl, obs_yt, obs_xr, obs_yt)

            if ((x1 > obs_xl and  x1 < obs_xr) and (y1 > obs_yb and y1 < obs_yt)):
                return True

            if ((x2 > obs_xl and  x2 < obs_xr)and (y2 > obs_yb and  y2 < obs_yt)):
                return True

            if (bottom or left or right or top):
                return True
        
        return False

    def lineIntersect(self, x1,  y1,  x2,  y2,  x3,  y3,  x4,  y4):

        # calculate the distance to intersection point
        uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
        uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

        # if uA and uB are between 0-1, lines are colliding
        if (uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1):
            return True
        return False
    

    def getNodes(self):
        return self.nodes;

    def getById(self,node_id):
        for i in self.nodes:
            if i.node_id == node_id:
                return i