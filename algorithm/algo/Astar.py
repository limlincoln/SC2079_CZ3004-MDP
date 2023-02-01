import settings
from algorithm.Entities import Obstacle
from Entities.Rectangle import Rectangle
class Astar:
    def __init__(self, grid, obstacles: list[Obstacle]):
        #20x20 grid
        self.grid = grid
        self.obstacles = obstacles
        self.targetLocations = self.generateTargetLocation(obstacles)
        print(self.targetLocations)

    def computePath(self):
        pass


    def generateTargetLocation(self, obstacles: list[Obstacle]):
        """
        get all the configurations that the robot needs to visit
        :param obstacles: List[Obstacles]
        :return:
            list of configurations in the form (x,y,direction in char)
        """
        targetLocations = []
        for ob in obstacles:
            if ob.imageOrientation == "right":
                targetLocations.append((ob.pos[0] + 50, ob.pos[0] - 10, 'R'))
            elif ob.imageOrientation == "top":
                targetLocations.append((ob.pos[0] + 10, ob.pos[1] + 50, 'T'))
            elif ob.imageOrientation == "left":
                targetLocations.append((ob.pos[0] - 50, ob.pos[1] + 10, 'L'))
            else:
                targetLocations.append((ob.pos[0] + 10, ob.pos[1] - 50, 'B'))
        return targetLocations

    def dubinsPathComputation(self, start, goal):
        pass

    def isWalkable(self, x,y, time=0):
        """
         Checks if the robot can occupy this location
        :param x: int
            x coordinate
        :param y: int
            y coordinate
        :param time: float

        :return:
        true if walkable
        """
        if x < 0 or x > self.grid[0] or y < 0 or y > self.grid[1]:
            return False
        robotRect = Rectangle((x,y), 'R')
        for obstacle in self.obstacles:
            if robotRect.isCollided(Rectangle(obstacle.pos, 'O')):
                return False

        return True



    def getRectCorners(self, pos, type):
        """
            get 4 corners of a rect based on the type
        :param pos: tuple
            in the form (x,y) representing the left bottom corner
        :param type: char
            'R' for robot and 'O' for obstacles
        :return:
            a tuple of the 4 corners in the order of topLeft->topRight->bottomRight->bottomLeft
        """
        corners = []
        if type == 'R':
            corners.append((pos[0], pos[1]+30))
            corners.append((pos[0]+30, pos[1]+30))
            corners.append((pos[0]+30, pos[1]))
            corners.append(pos)
        #this rectCorners is for obstacle avoidance
        elif type == 'O':
            ob_pos = (pos[0]+5, pos[1]+5)
            corners.append((ob_pos[0]-15, ob_pos[1]+15))
            corners.append((ob_pos[0]+15, ob_pos[1]+15))
            corners.append((ob_pos[0]+15, ob_pos[1]-15))
            corners.append((ob_pos[0]-15, ob_pos[1]-15))

        return corners