from algorithm.algo.Environment import AdvancedEnvironment
import itertools
from algorithm.algo.DubinsV2 import DubinsV2
from algorithm.algo.HStar import HybridAstar

class NearestNeighbour:
    def __init__(self, env: AdvancedEnvironment, start):
        self.env = env
        self.start = start
        self.targetLocations = env.targets
        self.dubins = DubinsV2(25, 10, self.env)
    def computeSequence(self):
        """
        returns the sequence of vertices to visit with minimum cost
        :return:
        sequence of verticles with their pos
        """
        permutations = list(itertools.permutations(self.targetLocations))
        lowestDistance = 9999999
        choice = []
        optimalPath = []
        for perm in permutations:
            distance = 0
            for i in range(len(perm)-1):
                distance += self.euclideanDistance(perm[i], perm[i+1])
            if distance <= lowestDistance:
                lowestDistance = distance
                path = self.findPath(perm)
                choice.append(path)
        if choice:
            print(choice)
            optimalPath = min(choice, key=lambda tup: tup[0][0])
        return optimalPath


    def euclideanDistance(self, start, end):

        return ((end[0]-start[0])**2 + (end[1]-start[1])**2)**0.5


    def findPath(self, targetLocations: list):
        """
        find dubins path
        :param targetLocations: list[tuple]
        :return: path
        """
        complete_path = []

        counter = 0
        start = self.start

        for ob in targetLocations:
            try:
                path = HybridAstar(self.env, self.dubins, start, ob, False)
                print(path)
            except:
                print("no path")
                break
            complete_path.append(path)
            start = next
            counter += 1

        if counter != len(targetLocations):
            print("path is incomplete!!")

        return complete_path




