from collections import deque


class Node:
    """
    Node of the RRT
    """
    def __init__(self, position, time, cost):
        self.destinationPointList = []
        self.position = position
        self.time = time
        self.cost = cost


class Edge:
    """
    Edge of the RRT
    """

    def __init__(self, startNode, endNode, path, cost):
        self.startNode = startNode
        self.endNode = endNode
        self.path = deque(path)
        self.cost = cost
