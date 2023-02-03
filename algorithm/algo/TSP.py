from algorithm.algo.Environment import StaticEnvironment
import itertools
from collections import deque
from algorithm.constants import DIRECTION
import numpy as np
from algorithm.algo.Astar import Astar


class NearestNeighbour:
    def __init__(self, env: StaticEnvironment, start):
        self.env = env
        self.targetLocations = self.env.generateTargetLocation()
        self.sequence = tuple()
        self.commands = deque()
        self.start = start

    def computeSequence(self):
        """
        returns the sequence of vertices to visit with minimum cost
        :return:
        sequence of verticles with their pos
        """
        permutations = list(itertools.permutations(self.targetLocations))
        costList = []
        for perm in permutations[1:10]:
            path = self.findPath(list(perm))

            if path:
                cost = self.calculateCost(path)
            else:
                cost = 9999999

            costList.append((path, cost))
        optimalPath = min(costList, key=lambda tup : tup[1])
        print("Optimal path" , optimalPath)
        return list(optimalPath[0])



    def euclideanDistance(self, start, end):
        return ((end[0]-start[0])**2 + (end[1]-start[1])**2)**0.5


    def findPath(self, targetLocations: list):
        path = []
        start = self.start
        counter = 0
        for ob in targetLocations:
            aStar = Astar(self.env, start, ob)
            if aStar.computePath() == None:
                print("no path found!!")
                break
            path.extend(aStar.getPath())
            start = ob
            counter += 1

        if counter != 5:
            print("Path is incomplete!!!")
            path = []
        return path

    def calculateCost(self, path: list[tuple]):
        cost = 0
        path = list(path)
        print(path)
        for command in path:
            print(command)
            if command[1] == 'S' or command[1] == 'SV':
                cost += 1
            elif command[1] == 'R' or command[1] == 'L':
                cost += 5
        return cost


