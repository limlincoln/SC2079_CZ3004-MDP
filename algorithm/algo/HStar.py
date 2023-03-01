import time

import numpy as np

from algorithm.algo.Environment import AdvancedEnvironment
from algorithm.algo.DubinsV2 import DubinsV2
from queue import PriorityQueue
import algorithm.settings as settings
from algorithm.HelperFunctions import basic_angle
from algorithm.constants import DIRECTION
from algorithm.algo.CommandV2 import CommandV2


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
        self.precision = (2, 2, 1)

    def solve(self):
        """
        main algo
        :return:
        """
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
            if time.perf_counter() - clock > len(self.env.targets) + 1:
                print("inifinite loop break")
                return None
            priority, _, currentNode = frontier.get()
            if self.in_goal_region(currentNode.pos):
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
        """
        Get neighbouring states
        :param node:
        :return:
        """

        moves = []
        pos = node.pos
        moves.extend(self.motionsCommands(pos))
        path = self.dubins.compute_best(pos, self.goal)
        if path:
            moves.append(Node(path[0][1], path[0][0], 1, path[1]))
        return moves

    def generateState(self, pos, t, delta):
        points = []
        current = pos
        length = (2 * np.pi * 20) / 4
        if t == 'S' or t == 'Z':
            length = 10
        for x in np.arange(0, length, delta):
            current = self.nextPos(current, t, delta)
            points.append(current)
        final_pos = self.generateEndState(pos, t)
        return points, final_pos

    def nextPos(self, pos, type, delta):
        """
        Get the next position in continuous step
        :param pos:
        :param v:
        :param steeringAngle:
        :return:
        """

        if type == "S":
            new_X = pos[0] + delta * np.cos(pos[2])
            new_Y = pos[1] + delta * np.sin(pos[2])
            new_orientation = pos[2]
        elif type == "R":
            new_X = pos[0] + delta * np.cos(pos[2])
            new_Y = pos[1] + delta * np.sin(pos[2])
            new_orientation = basic_angle(pos[2] - delta / 20)
        elif type == "L":
            new_X = pos[0] + delta * np.cos(pos[2])
            new_Y = pos[1] + delta * np.sin(pos[2])
            new_orientation = basic_angle(pos[2] + delta / 20)
        elif type == "Z":
            new_X = pos[0] - delta * np.cos(pos[2])
            new_Y = pos[1] - delta * np.sin(pos[2])
            new_orientation = pos[2]
        elif type == "RR":
            new_X = pos[0] - delta * np.cos(pos[2])
            new_Y = pos[1] - delta * np.sin(pos[2])
            new_orientation = basic_angle(pos[2] + delta / 20)
        elif type == "RL":
            new_X = pos[0] - delta * np.cos(pos[2])
            new_Y = pos[1] - delta * np.sin(pos[2])
            new_orientation = basic_angle(pos[2] - delta / 20)
        return new_X, new_Y, new_orientation

    def extract_path(self, backtrack, goalNode, startNode, dubinsNode=None):
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
        #path.append(startNode)
        path.reverse()
        self.path = path

    def motionsCommands(self, pos):
        moves = []
        available_moves = {'L': (pos, 10, 20),
                           'R': (pos, 10, 20),
                           'Z': (pos, -10, 25),
                           'RR': (pos, -10, 30),
                           'S': (pos, 10, 10),
                           'RL': (pos, -10, 30)}
        for key in available_moves:
            add = True
            move = available_moves[key]
            points, final_pos = self.generateState(move[0], key, 0.5)
            for point in points:
                if not self.env.isWalkable(point):
                    add = False
                    break
            if add:
                moves.append(Node(final_pos, key, move[2], points))
        return moves

    def rounding(self, pos):

        return np.round(pos[0]), np.round(pos[1]), np.round(pos[2], 6)

    def nextPosDiscrete(self, pos, v, angle):
        new_x = pos[0] + v * np.cos(pos[2] - angle)
        new_y = pos[0] + v * np.sin(pos[2] - angle)
        new_angle = basic_angle(pos[2] - angle)

        return round(new_x), round(new_y), new_angle

    def in_goal_region(self, pos):
        for i, value in enumerate(pos):
            if i <= 2:
                if abs(self.goal[i] - value) > self.precision[i]:
                    return False
        return True

    def generateEndState(self, pos, t):
        command = CommandV2(pos, 20, 10)
        if t == 'S':
            return command.moveStraight()
        elif t == 'Z':
            return command.moveRevese()
        elif t == "R":
            return command.moveRight()
        elif t == 'L':
            return command.moveLeft()
        elif t == 'RL':
            return command.faceLeftReverse()
        elif t == 'RR':
            return command.faceRightReverse()

