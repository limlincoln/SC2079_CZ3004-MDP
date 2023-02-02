from algo.Environment import StaticEnvironment
import queue
class NearestNeighbour:
    def __init__(self, env: StaticEnvironment, start, rrt, aStar):
        self.targetLocations = env.generateTargetLocation()
        self.visited = []
        self.queue = queue.PriorityQueue(5)
        self.start = start
        self.rrt = rrt
        self.aStar = aStar

    def computeSequence(self):
        """
        returns the sequence of vertices to visit. (possibly together with the path ?)
        :return:
        sequence of verticles with their pos
        """
        current = self.start
        self.queue.put(0, current)
        while not self.queue.empty():
            for node in self.targetLocations:
                if node not in self.visited:
                    pass




