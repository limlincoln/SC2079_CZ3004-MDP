import algorithm.constants as constants
from algorithm.algo.Command import Command
from algorithm.algo.Environment import StaticEnvironment
from algorithm.algo.Dubins import dist
from queue import PriorityQueue
class Astar:
    def __init__(self, env: StaticEnvironment, start, end):
        """
        :param env: Static Environment
        :param start: tuple (x,y,pos) in grid format
        :param end: tuple (x,y,pos) in grid format
        """
        self.env = env
        self.start = start
        self.end = end
        self.path = []

    def getNeighbours(self, pos):
        """
        Get next position relative to pos
        a fix distance of 5 when travelling straight
        robot will always make a 90 degrees turn
        :param pos: tuple (x,y,direction in rads)
        :return: list[nodes]
        """
        neighbours = []
        command = Command(pos)
        commandList = command.getCommands()
        turnPenalty = constants.COST.TURN_COST
        timeCost = constants.COST.MOVE_COST

        for index, c in enumerate(commandList):
            if self.env.isWalkable(c[0], c[1], 0):
                if index == constants.MOVEMENT.RIGHT.value or index == constants.MOVEMENT.LEFT.value:
                    "working"
                    neighbours.append((c, turnPenalty+50))
                else:
                    neighbours.append((c, timeCost))
        return neighbours

    def heuristic(self, pos, end):
        """

        :param pos: tuple
        :param end: tuple
        :return:
        distance between 2 points
        """
        return dist(pos, end)


    def computePath(self):
        """
        YOLO A star attempt
        :return:
        """
        frontier = PriorityQueue()
        backtrack = dict()
        cost = dict()
        goalNode = self.end
        startNode = self.start

        offset = 0 # dk for what
        frontier.put((0, offset, startNode))
        cost[startNode] = 0

        backtrack[startNode] = None

        while not frontier.empty():
            priority, _, currentNode = frontier.get()
            if currentNode[:3] == goalNode[:3]:
                self.extractCommands(backtrack, currentNode)
                return currentNode

            for newNode, weight in self.getNeighbours(currentNode):

                newCost = cost[currentNode] + weight
                print(newCost)

                if newNode not in backtrack or newCost < cost[newNode]:
                    offset += 1
                    priority = newCost + self.heuristic((newNode[0], newNode[1]), (goalNode[0], goalNode[1]))
                    backtrack[newNode] = currentNode
                    frontier.put((priority, offset, newNode))
                    cost[newNode] = newCost

        return None

    def extractCommands(self, backtrack, goalNode):
        """
        Extract dem commands to get to destination
        :param backtrack: dist
        :param goalNode: tuple
        :return:
        yolo
        """
        commands = []
        current = goalNode
        while current:
            current = backtrack.get(current, None)
            if current:
                commands.append(current)
        commands.pop()
        #commands.append(goalNode)
        commands.reverse()
        self.path.extend(commands)

    def getPath(self):
        return self.path
    def getCommandPath(self):
        commandList = [x[2:4] for x in self.path]
        return commandList