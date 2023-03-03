from algorithm.constants import DIRECTION
import settings


class CommandV2:
    def __init__(self, pos, radius, straight):
        self.radius = radius
        self.straight = straight
        self.pos = pos
        self.commands = []
        # needed to find the next direction
        self.dirList = [DIRECTION.TOP.value, DIRECTION.RIGHT.value, DIRECTION.BOTTOM.value, DIRECTION.LEFT.value]
        # future use
        self.diaDirList = [DIRECTION.NORTHEAST, DIRECTION.SOUTHEAST, DIRECTION.SOUTHWEST, DIRECTION.NORTHWEST]

    def getCommands(self):
        """
        commands populator
        :return: list(tuple)
        """
        self.moveStraight()
        self.moveLeft()
        self.moveRight()
        self.faceLeftReverse()  # reverse right
        self.faceRightReverse()  # reverse left
        # self.turnOntheSpot()
        return self.commands

    def moveStraight(self):
        """
        Move front and back
        :return:
        new pos forward and backward
        """
        if self.pos[2] == DIRECTION.TOP.value:
            return self.pos[0], self.pos[1] + self.straight, self.pos[2]

        elif self.pos[2] == DIRECTION.LEFT.value:
            return self.pos[0] - self.straight, self.pos[1], self.pos[2]

        elif self.pos[2] == DIRECTION.RIGHT.value:
            return self.pos[0] + self.straight, self.pos[1], self.pos[2]
        else:
            return self.pos[0], self.pos[1] - self.straight, self.pos[2]

    def moveRevese(self):
        if self.pos[2] == DIRECTION.TOP.value:
            return self.pos[0], self.pos[1] - self.straight, self.pos[2]

        elif self.pos[2] == DIRECTION.LEFT.value:
            return self.pos[0] + self.straight, self.pos[1], self.pos[2]

        elif self.pos[2] == DIRECTION.RIGHT.value:
            return self.pos[0] - self.straight, self.pos[1], self.pos[2]
        else:
            return self.pos[0], self.pos[1] + self.straight, self.pos[2]

    def moveRight(self):
        """
        turn right 90* (for now)
        :return:
        new pos after moving right
        """
        if self.pos[2] == DIRECTION.TOP.value:
            return self.pos[0] + self.radius, self.pos[1] + self.radius, self.dirList[
                (self.dirList.index(self.pos[2]) + 1) % 4]
        elif self.pos[2] == DIRECTION.LEFT.value:
            return self.pos[0] - self.radius, self.pos[1] + self.radius, self.dirList[
                (self.dirList.index(self.pos[2]) + 1) % 4]
        elif self.pos[2] == DIRECTION.RIGHT.value:
            return self.pos[0] + self.radius, self.pos[1] - self.radius, self.dirList[
                (self.dirList.index(self.pos[2]) + 1) % 4]
        else:
            return self.pos[0] - self.radius, self.pos[1] - self.radius, self.dirList[
                (self.dirList.index(self.pos[2]) + 1) % 4]

    def moveLeft(self):
        """
        turn left 90* (for now)
        :return:
        new pos after moving left
        """
        if self.pos[2] == DIRECTION.TOP.value:

            return self.pos[0] - self.radius, self.pos[1] + self.radius, self.dirList[
                self.dirList.index(self.pos[2]) - 1]
        elif self.pos[2] == DIRECTION.LEFT.value:

            return self.pos[0] - self.radius, self.pos[1] - self.radius, self.dirList[
                abs(self.dirList.index(self.pos[2]) - 1) % 4]
        elif self.pos[2] == DIRECTION.RIGHT.value:
            return self.pos[0] + self.radius, self.pos[1] + self.radius, self.dirList[
                abs(self.dirList.index(self.pos[2]) - 1) % 4]
        else:
            return self.pos[0] + self.radius, self.pos[1] - self.radius, self.dirList[
                abs(self.dirList.index(self.pos[2]) - 1) % 4]

    def faceRightReverse(self):

        """
        90 degrees full lock reverse
        :return: tuple
        commands based on the current direction
        """

        if self.pos[2] == DIRECTION.TOP.value:
            return self.pos[0] + self.radius, self.pos[1] - self.radius, self.dirList[
                self.dirList.index(self.pos[2]) - 1]
        elif self.pos[2] == DIRECTION.LEFT.value:
            return self.pos[0] + self.radius, self.pos[1] + self.radius, self.dirList[
                abs(self.dirList.index(self.pos[2]) - 1) % 4]
        elif self.pos[2] == DIRECTION.RIGHT.value:
            return self.pos[0] - self.radius, self.pos[1] - self.radius, self.dirList[
                abs(self.dirList.index(self.pos[2]) - 1) % 4]
        else:
            return self.pos[0] - self.radius, self.pos[1] + self.radius, self.dirList[
                abs(self.dirList.index(self.pos[2]) - 1) % 4]

    def faceLeftReverse(self):
        """
        90 degress full lock reverse
        :return: tuple
        commands based on the current direction
        """

        if self.pos[2] == DIRECTION.TOP.value:
            return self.pos[0] - self.radius, self.pos[1] - self.radius, self.dirList[
                (self.dirList.index(self.pos[2]) + 1) % 4]

        elif self.pos[2] == DIRECTION.LEFT.value:
            return self.pos[0] + self.radius, self.pos[1] - self.radius, self.dirList[
                (self.dirList.index(self.pos[2]) + 1) % 4]

        elif self.pos[2] == DIRECTION.RIGHT.value:

            return self.pos[0] - self.radius, self.pos[1] + self.radius, self.dirList[
                (self.dirList.index(self.pos[2]) + 1) % 4]
        else:
            return self.pos[0] + self.radius, self.pos[1] + self.radius, self.dirList[
                (self.dirList.index(self.pos[2]) + 1) % 4]
