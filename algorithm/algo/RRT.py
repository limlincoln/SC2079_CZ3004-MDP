from algorithm.algo.Environment import StaticEnvironment
from algorithm.algo.NodeNEdges import Node, Edge
from algorithm.algo.Dubins import Dubins, dist
import numpy as np
from algorithm.constants import DIRECTION
class RRT:
    def __init__(self, environment : StaticEnvironment, precision=(5, 5, 1)):
        self.environment = environment
        self.targetLocations = self.environment.getTargetLocation()
        self.nodes = {}
        self.edges = {}
        self.localPlanner = Dubins(25, 1)
        self.root = (0, 0, 0)
        self.goal = (0, 0, 0)
        self.precision = precision

    def setStart(self, start):
        """
        reset graph
        :param start:  tuple
        in the form of (x,y,direction in rads)
        :return:
        """
        self.nodes = {}
        self.edges = {}
        self.nodes[start] = Node(start, 0, DIRECTION.TOP.value)
        self.root = start



    def computePath(self, goal, nbIteration=100, goalRate=0.1, metric='local'):
        """
        Executes the algorithm with an empty graph, with the start postion
        :param goal: tuple
        :param nbIteration: int
        the number of maximal iterations
        :param goalRate: float
        probaility to expand towards the goal rather than towards a randomly selected sample
        :param metric: string
        'local' for dubins and 'euclidian' for euclidian
        :return:
        """
        assert len(goal) == len(self.precision)
        self.goal = goal

        for _ in range(nbIteration):
            if np.random.rand() > 1 - goalRate:
                sample = goal
            else:
                sample = self.environment.randomFreeSpace()
        options = self.selectOptions(sample, 10, metric)
        for node, option in options:
            if option[0] == float('inf'):
                break
            path = self.localPlanner.generatePoints(node, sample, option[1], option[2])

            for i, point in enumerate(path):
                if not self.environment.isWalkable(point[0], point[1],
                                                   self.nodes[node].time+i):
                    break
            else:
                self.nodes[sample] = Node(sample, self.nodes[node].time+option[0],
                                          self.nodes[node].cost+option[0])
                self.nodes[node].destinationPointList.append(sample)
                self.edges[node, sample] = Edge(node, sample, path, option[0])
                if self.inGoalRegion(sample):
                    return
                break


    def selectOptions(self, sample, nbOptions, metric='local'):
        """
        Choose the best nodes for expansion of the tree and returns them in a list ordered by increasing cost.

        :param sample: tuple
        :param nbOptions: int
            the number of options requested
        :param metric: str
        :return:
        options : list
        """
        if metric == 'local':
            options = []
            for node in self.nodes:
                options.extend(
                    [(node, opt) for opt in self.localPlanner.computeAllPath(node, sample)]
                )
                options.sort(key=lambda x: x[1][0])
                options = options[:nbOptions]
        else:
            options = [(node, dist(node, sample)) for node in self.nodes]
            options.sort(key=lambda x: x[1])
            options = options[:nbOptions]
            newOpt = []
            for node, _ in options:
                dbOptions = self.localPlanner.computeAllPath(node, sample)
                newOpt.append((node, min(dbOptions, key=lambda x: x[0])))
            options = newOpt
        return options
    def inGoalRegion(self, sample):
        for i, value in enumerate(sample):
            if abs(self.goal[i]-value > self.precision[i]):
                return False
        return True
    def selectBestEdge(self):
        """
        Selects the best edge of the tree among the ones leaving from the root
        :return:
        the best edge
        """
        node = max([(child, self.childrenCount(child)) for child in self.nodes[self.root].destinationPointList],
                   key= lambda x: x[1])[0]
        bestEdge = self.edges[(self.root, node)]
        for child in self.nodes[self.root].destinationPointList:
            if child == node:
                continue
            self.edges.pop((self.root, child))
            self.deleteAllChildren(child)
        self.nodes.pop(self.root)
        self.root = node
        return bestEdge

    def deleteAllChildren(self, node):
        """
        Removesall the nodes of the tree below the requested node
        :param node: node
        :return:
        """
        if self.nodes[node].destinationPointList:
            for child in self.nodes[node].destinationPointList:
                self.edges.pop((node, child))
                self.deleteAllChildren(child)
        self.nodes.pop(node)

    def childrenCount(self,node):
        """
        high time complexity atm need some optimisation
        :param node: node
        :return:
        """
        if not self.nodes[node].destinationPointList:
            return 0
        total = 0
        for child in self.nodes[node].destinationPointList:
            total += 1 + self.childrenCount(child)

        return total

    def getRectCorners(self, pos, type):
        """
            get 4 corners of a rect based on the type
        :param pos: tuple
            in the form (x,y) representing the left bottom corner
        :param type: char
            'R' for robot and 'O' for obstacles
        :return:
            a tuple of the 4 corners in the order of topLeft->topRight->bottomRight->bottomLeft
        """
        corners = []
        if type == 'R':
            corners.append((pos[0], pos[1]+30))
            corners.append((pos[0]+30, pos[1]+30))
            corners.append((pos[0]+30, pos[1]))
            corners.append(pos)
        #this rectCorners is for obstacle avoidance
        elif type == 'O':
            ob_pos = (pos[0]+5, pos[1]+5)
            corners.append((ob_pos[0]-15, ob_pos[1]+15))
            corners.append((ob_pos[0]+15, ob_pos[1]+15))
            corners.append((ob_pos[0]+15, ob_pos[1]-15))
            corners.append((ob_pos[0]-15, ob_pos[1]-15))

        return corners

    def getPath(self, nodes=False):
        """
        return the path of nodes
        :param nodes: bool to display nodes or not
        :return:
        """
        nodes = []
        path = []
        print(self.nodes)
        print(self.edges)
        if nodes and self.nodes:
            nodes = np.array(list(self.nodes.keys()))

        for _, val in self.edges.items():
            if val.path:
                path = np.array(val.path)

        return path, nodes

