from algorithm.Entities.Obstacle import Obstacle
class VirtualRect:
    """
    Class representing virtual obstacle that the robot cannot move to.
    """
    def __init__(self, ob: Obstacle):
        """
        Class representing virtual obstacle that the robot cannot move to.
        :param ob: Obstacle
        """
        self.pos = (ob.pos[0] - 15, ob.pos[1] + 25)
        self.length = 40

    def isCollided(self, robotPos):
        """
        Check if the center point of the robot collide with the rect provided
        :param robotPos: tuple (x,y)
        :return: bool
        """

        if(self.pos[0] <= robotPos[0] <= self.pos[0] + self.length and
                self.pos[1] <= robotPos[1] <= self.pos[1] + self.length):
            return True
        return False


