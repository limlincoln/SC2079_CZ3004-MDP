import numpy as np

from algorithm.algo.Environment import AdvancedEnvironment
from algorithm.algo.DubinsV2 import DubinsV2
from queue import PriorityQueue

class Node:

    def __init__(self, pos, moves, cost):
        self.pos = pos
        self.moves = moves
        self.cost = cost
class HybridAstar:
    def __init__(self, env: AdvancedEnvironment,dubins: DubinsV2, start, goal, reverse):
        self.env = env
        self.start = start
        self.goal = goal
        self.dubins = dubins
        self.path = []


    def solve(self):
        frontier = PriorityQueue()
        backtrack = dict()
        cost = dict()
        goalNode : Node= Node(self.goal, 0, 0)
        startNode : Node = Node(self.start, 0, 0)

        offset = 0

        frontier.put((0, offset, startNode))
        cost[startNode] = 0
        backtrack[startNode] = None

        while not frontier.empty():

            priority, _, currentNode = frontier.get()
            if currentNode.pos == goalNode.pos:
                self.extract_path(backtrack, currentNode, startNode)
                return currentNode

            for newNode in self.get_neighbours(currentNode):
                newCost = cost[currentNode] + newNode.cost
                if newNode not in backtrack or newCost < cost[newNode]:
                    cost[newNode] = newCost
                    offset +=1
                    priority = newCost + self.heuristic(newNode.pos, goalNode.pos)
                    backtrack[newNode] = currentNode
                    frontier.put((priority, offset, newNode))

        return None




    def heuristic(self, start, end):
        self.dubins.distCenter(start, end)

    def get_neighbours(self, node: Node):

        moves = []
        pos = node.pos
        available_moves =  {'L': (pos, 0.4, 10, 0.05),
                            'R': (pos, -0.4, 10, 0.05),
                            'Z': (pos, 0.4, -10, 0.2),
                            'X': (pos, -0.4, -10, 0.2),
                            'S': (pos, 0, 10, 0),
                            'v': (pos, 0, -10, 0.1)}
        for key in available_moves:
            move = available_moves[key]
            cost = move[3]
            new_pos = self.nextPos(move[0], move[1], move[2])
            if self.env.isWalkable(new_pos):
                moves.append(Node(new_pos, key, cost))

        """
        # turn left
        moves.append(Node(self.nextPos(pos, 0.4, 10), 'L', 0.05))
        # turn right
        moves.append(Node(self.nextPos(pos, -0.4, 10), 'R', 0.05))
        # reverse left
        moves.append(Node(self.nextPos(pos, 0.4, -10), 'Z', 0.2))
        # reverse right
        moves.append(Node(self.nextPos(pos, -0.4, -10), 'X',0.2))
        #move straight
        moves.append(Node(self.nextPos(pos, 0, 10), 'S', 0))
        #move reverse
        moves.append(Node(self.nextPos(pos, 0, -10), 'V', 0.1))
        """

        #dubins path
        path = self.dubins.compute_best(pos, self.goal)
        if path:
            moves.append(Node(path[0][1], (path[0][0], path[1]), 0))

        return moves





    def nextPos(self, pos, change, v):
        new_angle = pos[3] + (v/self.dubins.radius) * np.tan(change)
        new_x = pos[0] + v*np.cos(new_angle)
        new_y = pos[1] + v*np.sin(new_angle)

        return new_x, new_y, new_angle

    def extract_path(self, backtrack, goalNode, startNode):
        """

        :param startNode: Node
        :param backtrack: dist
        :param goalNode: Node
        :return:
        """

        path = []
        current = goalNode

        while current != startNode:
            path.append(current)
            current = backtrack[current]
        path.append(startNode)
        path.reverse()
        self.path  = path
