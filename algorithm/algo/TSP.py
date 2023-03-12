
from algorithm.algo.Environment import StaticEnvironment
import itertools
from collections import deque
from algorithm.algo.Astar import Astar
from algorithm.constants import DIRECTION

class NearestNeighbour1:
    def __init__(self, env: StaticEnvironment, start):
        self.env = env
        self.targetLocations = self.env.getTargetLocation()
        self.sequence = tuple()
        self.commands = deque()
        self.start = start
        self.optimalPathWithCoords = None
        self.commandList = None
        self.simulatorCommandList = []

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
                lowestDistance = distance
                stmPath, path = self.findPath(list(perm))
                cost = self.calculateCost(stmPath)

                costList.append((stmPath, path, cost))
            else:
                continue
        optimalPath = min(costList, key=lambda tup: tup[2])
        print("Stm path", optimalPath[0])
        print("coords", optimalPath[1])
        simCoords = optimalPath[1].copy()
        coords = self.convert_to_coords(optimalPath[1])
        self.commandList = list(optimalPath[0]), coords
        print(("rpi path", self.commandList))
        self.optimalPathWithCoords = simCoords

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
                if command == 's' or command == 'b':
                    cost += 1
                elif command == 'd' or command == 'u':
                    cost += 8
                elif command == 'OL' or command == 'OR':
                    cost += 10
                elif command == 'v' or command == 'w':
                    cost += 10
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
        commands = []
        for x in path:
            if x[3] is not "P":
                commands.extend(x[3])
        return commands






    def getOptimalWithCoords(self):
        return self.optimalPathWithCoords

    def getCommandList(self):
        return self.commandList

    def convert_to_coords(self, path):
        coords_path = []

        for x in path:
            coords_path.append(self.convertTuple( ("ROBOT",str(x[0]//10), str(x[1]//10), self.convert_direction(x[2].name) )) )

        return coords_path


    def convert_direction(self,f):
        if f == "TOP":
            return "N"
        elif f == "BOTTOM":
            return "S"
        elif f == "RIGHT":
            return "E"
        else:
            return "W"

    def convertTuple(self, tup):
        str = ', '.join(tup)
        return str

    def convert_to_simulator_commands(self):
        commands = []
        for x in self.commandList[0]:
            commands.extend(x[1].split(','))
        return commands


