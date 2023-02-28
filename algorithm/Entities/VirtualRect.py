from algorithm.Entities.Obstacle import Obstacle
from shapely import geometry


class VirtualRect:
    """
    Class representing virtual obstacle that the robot cannot move to.
    """

    def __init__(self, ob: Obstacle, task=1):
        """
        Class representing virtual obstacle that the robot cannot move to.
        :param ob: Obstacle
        """
        self.pos = (ob.pos[0] - 10, ob.pos[1] + 20)
        self.length = 30
        if task == 1:
            self.polygon = geometry.box(self.pos[0], self.pos[1] - self.length, self.pos[0] + self.length, self.pos[1])
        elif task == 2:
            self.length = 60
            self.width = 10
            self.polygon = geometry.box(self.pos[0], self.pos[1]- self.length, self.pos[0] + self.width, self.pos[1])
        self.center = (self.pos[0] + 15, self.pos[1] - 15)

    def isCollided(self, robotPos):
        """
        Check if the center point of the robot collide with the rect provided
        :param robotPos: tuple (x,y)
        :return: bool
        """

        if self.pos[0] <= robotPos[0] <= (self.pos[0] + self.length) and self.pos[1] <= robotPos[1] <= self.pos[
            1] + self.length:
            return True

        return False

    def collides(self, pos):
        """
        check if collide with given point
        :param pos:
        :return:
        """
        return self.polygon.contains(geometry.Point(pos[0], pos[1]))
