
from algorithm.algo.Environment import StaticEnvironment
import itertools
from collections import deque
from algorithm.algo.Astar import Astar


class NearestNeighbour:
    def __init__(self, env: StaticEnvironment, start):
        self.env = env
        self.targetLocations = self.env.getTargetLocation()
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
                stmPath, path = self.findPath(list(perm))
                cost = self.calculateCost(stmPath)
                costList.append((stmPath, path, cost))
            else:
                continue
        optimalPath = min(costList, key=lambda tup: tup[2])
        print("Stm path", optimalPath[0])
        print("coords", optimalPath[1])
        self.commandList = list(optimalPath[0])
        self.optimalPathWithCoords = optimalPath[1]

    def euclideanDistance(self, start, end):

        return ((end[0]-start[0])**2 + (end[1]-start[1])**2)**0.5


    def findPath(self, targetLocations: list):
        """

        :param targetLocations: list[tuple]
        :return: stmPath and path
        """
        path = []
        stmPath = []
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
            cPath = aStar.getSTMCommands()
            stmPath.append((self.env.obID[self.env.getTargetLocation().index(ob)], cPath))
            start = next
            counter += 1
            """
            """
        if counter != len(targetLocations):
            print("Path is incomplete!!!")
            # path = []
            # stmPath = []
        return stmPath, path

    def calculateCost(self, path: list[tuple]):
        cost = 0
        diff = len(self.env.obID) - len(path)
        cost += (999 * diff)
        for tup in path:
            for command in tup[1]:
                if command == 'S' or command == 'SV':
                    cost += 1
                elif command == 'R' or command == 'L':
                    cost += 8
                elif command == 'OL' or command == 'OR':
                    cost += 10
                elif command == 'RR' or command == 'RL':
                    cost += 8
                elif command == '3P':
                    cost += 10
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

    def getSTMCommands(self, path):
        STMCommands = [x[3] for x in path]
        return STMCommands






    def getOptimalWithCoords(self):
        return self.optimalPathWithCoords

    def getCommandList(self):
        return self.commandList

