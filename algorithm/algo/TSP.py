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
        self.optimalPathWithCoords = None
        self.commandList = None

    def computeSequence(self):
        """
        returns the sequence of vertices to visit with minimum cost
        :return:
        sequence of verticles with their pos
        """
        permutations = list(itertools.permutations(self.targetLocations))
        costList = []
        lowestDistance = 999999
        for perm in permutations:
            distance = 0
            for i in range(len(perm)-1):
                distance += self.euclideanDistance(perm[i], perm[i+1])

            if distance <= lowestDistance:
                print(distance)
                lowestDistance = distance
                commandPath, path = self.findPath(list(perm))
                if commandPath:
                    cost = self.calculateCost(commandPath)
                else:
                    cost = 9999999
                costList.append((commandPath, path, cost))
            else:
                continue
        optimalPath = min(costList, key=lambda tup: tup[2])
        print("Optimal path" , optimalPath[0])
        print("coords", optimalPath[1])
        self.commandList = list(optimalPath[0])
        self.optimalPathWithCoords = optimalPath[1]



    def euclideanDistance(self, start, end):
        return ((end[0]-start[0])**2 + (end[1]-start[1])**2)**0.5


    def findPath(self, targetLocations: list):
        path = []
        commandPath = []
        start = self.start
        counter = 0
        for ob in targetLocations:
            aStar = Astar(self.env, start, ob)
            next = aStar.computePath()
            if next == None:
                print("no path found!!")
                break
            newPath = aStar.getPath()
            path.extend(newPath)
            cPath = aStar.getCommandPath()
            commandPath.extend(cPath)
            start = next
            counter += 1
            """
            """
        if counter != 5:
            print("Path is incomplete!!!")
            path = []
            commandPath = []
        return commandPath, path

    def calculateCost(self, path: list[tuple]):
        cost = 0
        for command in path:
            if command[1] == 'S' or command[1] == 'SV':
                cost += 1
            elif command[1] == 'R' or command[1] == 'L':
                cost += 5
            elif command[1] == 'OL' or command[1] == 'OR':
                cost += 10
            elif command[1] == 'RR' or command[1] == 'RL':
                cost += 5
        return cost

    def convertToCommands(self, path):
        commandList = []
        prev = path[0][1]
        counter = 0
        for x in path:
            if x[1] == prev:
                counter += 1
            else:
                string = str(counter) + prev
                commandList.append(string)
                counter = 1
            prev = x[1]

        return commandList





    def getOptimalWithCoords(self):
        return self.optimalPathWithCoords

    def getCommandList(self):
        return self.commandList