import time

import numpy as np

from algorithm.algo.Environment import AdvancedEnvironment
from algorithm.algo.DubinsV2 import DubinsV2
from queue import PriorityQueue
import algorithm.settings as settings
from algorithm.HelperFunctions import basic_angle
class Node:

    def __init__(self, pos, moves, cost, path):
        self.pos = pos
        self.moves = moves
        self.cost = cost
        self.path = path

class HybridAstar:
    def __init__(self, env: AdvancedEnvironment, dubins: DubinsV2, start, goal, reverse):
        self.env = env
        self.start = start
        self.goal = goal
        self.dubins = dubins
        self.path = []
        self.L = 1

    def solve(self):
        clock = time.perf_counter()
        frontier = PriorityQueue()
        backtrack = dict()
        cost = dict()
        goalNode: Node = Node(self.goal, [], 0, [])
        startNode: Node = Node(self.start, [], 0, [])
        offset = 0

        frontier.put((0, offset, startNode))
        cost[startNode] = 0
        backtrack[startNode] = None
        while not frontier.empty():
            if time.perf_counter() - clock > 20:
                print("inifinite loop break")
                return None
            priority, _, currentNode = frontier.get()
            if self.rounding(currentNode.pos) == self.rounding(goalNode.pos):
                self.extract_path(backtrack, currentNode, startNode)
                return currentNode
            for newNode in self.get_neighbours(currentNode):
                newCost = cost[currentNode] + newNode.cost
                if newNode not in backtrack or newCost < cost[newNode]:
                    cost[newNode] = newCost
                    offset += 1
                    priority = newCost + self.heuristic(newNode.pos, goalNode.pos)
                    backtrack[newNode] = currentNode
                    frontier.put((priority, offset, newNode))
        return None

    def heuristic(self, start, end):
        return self.dubins.distCenter(start, end)

    def get_neighbours(self, node: Node):

        moves = []
        pos = node.pos
        moves.extend(self.motionCommandsDiscrete(pos))
        path = self.dubins.compute_best(pos, self.goal)
        if path:
            moves.append(Node(path[0][1], path[0][0], 1, path[1]))
        return moves

    def nextPos(self, pos, v, steeringAngle):
        """
        Get the next position in continuous step
        :param pos:
        :param v:
        :param steeringAngle:
        :return:
        """
        new_x = pos[0] + v * np.cos(pos[2])
        new_y = pos[1] + v * np.sin(pos[2])
        new_angle = basic_angle(pos[2] + v*np.tan(steeringAngle))
        return new_x, new_y, new_angle

    def extract_path(self, backtrack, goalNode, startNode, dubinsNode = None):
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
        self.path = path


    def motionsCommands(self, pos):
        moves = []
        available_moves = {'L': (pos, settings.MAX_STEERING_ANGLE, 10, 5),
                           'R': (pos, -settings.MAX_STEERING_ANGLE, 10, 5),
                           'Z': (pos, settings.MAX_STEERING_ANGLE, -10, 20),
                           'X': (pos, -settings.MAX_STEERING_ANGLE, -10, 20),
                           'S': (pos, 0, 10, 2),
                           'v': (pos, 0, -10, 10)}
        for key in available_moves:
            move = available_moves[key]
            cost = move[3]
            new_pos = self.nextPos(move[0], move[2], move[1])
            if self.env.isWalkable(new_pos):
                moves.append(Node(new_pos, key, cost, []))
        return moves

    def motionCommandsDiscrete(self, pos):
        moves = []
        available_moves = {'L': (pos, np.pi, 10, 5),
                           'R': (pos, -np.pi, 10, 5),
                           'Z': (pos, np.pi, -10, 20),
                           'X': (pos, -np.pi, -10, 20),
                           'S': (pos, 0, 10, 2),
                           'v': (pos, 0, -10, 10)}
        for key in available_moves:
            move = available_moves[key]
            cost = move[3]
            new_pos = self.nextPosDiscrete(move[0], move[2], move[1])
            if self.env.isWalkable(new_pos):
                moves.append(Node(new_pos, key, cost, []))
        return moves

    def rounding(self, pos):

        return np.round(pos[0]), np.round(pos[1]), np.round(pos[2], 6)

    def nextPosDiscrete(self, pos, v, angle):
        new_x = pos[0] + v*np.cos(pos[2]-angle)
        new_y = pos[0] + v*np.sin(pos[2]-angle)
        new_angle = basic_angle(pos[2]-angle)

        return round(new_x) , round(new_y), new_angle
