
from algorithm.constants import DIRECTION
import settings


class Command:
    def __init__(self, pos):
        self.pos = pos
        self.commands = []

        # needed to find the next direction
        self.dirList = [DIRECTION.TOP, DIRECTION.RIGHT, DIRECTION.BOTTOM, DIRECTION.LEFT]
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
        self.faceLeftReverse() # reverse right
        self.faceRightReverse() # reverse left
        #self.turnOntheSpot()
        return self.commands

    def moveStraight(self):
        """
        Move front and back
        :return:
        new pos forward and backward
        """
        if self.pos[2] == DIRECTION.TOP:

            self.commands.append((self.pos[0], self.pos[1] + settings.SPEED_FACTOR, self.pos[2], 's'))
            self.commands.append((self.pos[0], self.pos[1] - settings.SPEED_FACTOR, self.pos[2], 'b'))

        elif self.pos[2] == DIRECTION.LEFT:
            self.commands.append((self.pos[0] - settings.SPEED_FACTOR, self.pos[1], self.pos[2], 's'))
            self.commands.append((self.pos[0] + settings.SPEED_FACTOR, self.pos[1], self.pos[2], 'b'))

        elif self.pos[2] == DIRECTION.RIGHT:
            self.commands.append((self.pos[0] + settings.SPEED_FACTOR, self.pos[1], self.pos[2], 's'))
            self.commands.append((self.pos[0] - settings.SPEED_FACTOR, self.pos[1], self.pos[2], 'b'))

        else:
            self.commands.append((self.pos[0], self.pos[1] - settings.SPEED_FACTOR, self.pos[2], 's'))
            self.commands.append((self.pos[0], self.pos[1] + settings.SPEED_FACTOR, self.pos[2], 'b'))

    def moveRight(self):
        """
        turn right 90* (for now)
        :return:
        new pos after moving right
        """
        if self.pos[2] == DIRECTION.TOP:
            self.commands.append((self.pos[0]+settings.TURNING_RADIUS_X, self.pos[1]+settings.TURNING_RADIUS_Y, self.dirList[(self.dirList.index(self.pos[2]) + 1) % 4], 'd'))
        elif self.pos[2] == DIRECTION.LEFT:
            self.commands.append((self.pos[0]-settings.TURNING_RADIUS_X, self.pos[1]+settings.TURNING_RADIUS_Y, self.dirList[(self.dirList.index(self.pos[2]) + 1) % 4], 'd'))
        elif self.pos[2] == DIRECTION.RIGHT:
            self.commands.append((self.pos[0]+settings.TURNING_RADIUS_X, self.pos[1]-settings.TURNING_RADIUS_Y, self.dirList[(self.dirList.index(self.pos[2]) + 1) % 4], 'd'))
        else:
            self.commands.append((self.pos[0]-settings.TURNING_RADIUS_X, self.pos[1]-settings.TURNING_RADIUS_Y, self.dirList[(self.dirList.index(self.pos[2]) + 1) % 4], 'd'))




    def moveLeft(self):
        """
        turn left 90* (for now)
        :return:
        new pos after moving left
        """
        if self.pos[2] == DIRECTION.TOP:
            self.commands.append(
                (self.pos[0]-settings.TURNING_RADIUS_X, self.pos[1]+settings.TURNING_RADIUS_Y, self.dirList[self.dirList.index(self.pos[2]) - 1],
                 'u'))
        elif self.pos[2] == DIRECTION.LEFT:
            self.commands.append(
                (self.pos[0]-settings.TURNING_RADIUS_X, self.pos[1]-settings.TURNING_RADIUS_Y, self.dirList[abs(self.dirList.index(self.pos[2]) - 1) % 4],
                 'u'))
        elif self.pos[2] == DIRECTION.RIGHT:
            self.commands.append(
                (self.pos[0]+settings.TURNING_RADIUS_X, self.pos[1]+settings.TURNING_RADIUS_Y, self.dirList[abs(self.dirList.index(self.pos[2]) - 1) % 4],
                 'u'))
        else:
            self.commands.append(
                (self.pos[0]+settings.TURNING_RADIUS_X, self.pos[1]-settings.TURNING_RADIUS_Y, self.dirList[abs(self.dirList.index(self.pos[2]) - 1) % 4],
                 'u'))

    def faceRightReverse(self):

        """
        90 degrees full lock
        reverse left
        :return: tuple
        commands based on the current direction
        """

        if self.pos[2] == DIRECTION.TOP:
            self.commands.append((self.pos[0]+settings.TURNING_RADIUS_X, self.pos[1]-settings.TURNING_RADIUS_Y, self.dirList[
                self.dirList.index(self.pos[2]) - 1], 'w'))
        elif self.pos[2] == DIRECTION.LEFT:
            self.commands.append((self.pos[0]+settings.TURNING_RADIUS_X, self.pos[1]+settings.TURNING_RADIUS_Y, self.dirList[(self.dirList.index(self.pos[2]) - 1) % 4], 'w'))
        elif self.pos[2] == DIRECTION.RIGHT:
            self.commands.append((self.pos[0]-settings.TURNING_RADIUS_X, self.pos[1]-settings.TURNING_RADIUS_Y, self.dirList[(self.dirList.index(self.pos[2]) - 1) % 4], 'w'))
        else:
            self.commands.append((self.pos[0]-settings.TURNING_RADIUS_X, self.pos[1]+settings.TURNING_RADIUS_Y, self.dirList[(self.dirList.index(self.pos[2]) - 1) % 4], 'w'))

    def faceLeftReverse(self):
        """
        90 degress full lock reverse
        :return: tuple
        commands based on the current direction
        """

        if self.pos[2] == DIRECTION.TOP:
            self.commands.append(
                (self.pos[0]-settings.TURNING_RADIUS_X, self.pos[1]-settings.TURNING_RADIUS_Y, self.dirList[(self.dirList.index(self.pos[2]) + 1)%4],
                 'v'))
        elif self.pos[2] == DIRECTION.LEFT:
            self.commands.append(
                (self.pos[0]+settings.TURNING_RADIUS_X, self.pos[1]-settings.TURNING_RADIUS_Y, self.dirList[abs(self.dirList.index(self.pos[2]) + 1) % 4],
                 'v'))
        elif self.pos[2] == DIRECTION.RIGHT:
            self.commands.append(
                (self.pos[0]-settings.TURNING_RADIUS_X, self.pos[1]+settings.TURNING_RADIUS_Y, self.dirList[abs(self.dirList.index(self.pos[2]) + 1) % 4],
                 'v'))
        else:
            self.commands.append(
                (self.pos[0]+settings.TURNING_RADIUS_X, self.pos[1]+settings.TURNING_RADIUS_Y, self.dirList[abs(self.dirList.index(self.pos[2]) + 1) % 4],
                 'v'))

    def turnOntheSpot(self):
        self.commands.append((
            self.pos[0], self.pos[1], self.dirList[abs(self.dirList.index(self.pos[2]) + 1) % 4]
        , 'OR'))

        self.commands.append((
            self.pos[0], self.pos[1], self.dirList[abs(self.dirList.index(self.pos[2]) - 1) % 4]
        , "OL"))

    def threePointTurn(self):
        if self.pos[2] == DIRECTION.TOP:
            self.commands.append((
                self.pos[0],self.pos[1] - 90, self.dirList[(self.dirList.index(self.pos[2]) + 2) % 4], '3P'
            ))
        elif self.pos[2] == DIRECTION.RIGHT:
            self.commands.append((
                self.pos[0] - 90, self.pos[1], self.dirList[(self.dirList.index(self.pos[2]) + 2) % 4], '3P'
            ))
        elif self.pos[2] == DIRECTION.LEFT:
            self.commands.append((
                self.pos[0] + 90, self.pos[1], self.dirList[(self.dirList.index(self.pos[2]) + 2) % 4], '3P'
            ))
        elif self.pos[2] == DIRECTION.BOTTOM:
            self.commands.append((
                self.pos[0], self.pos[1] + 90, self.dirList[(self.dirList.index(self.pos[2]) + 2) % 4], '3P'
            ))
        elif self.pos[2] == DIRECTION.NORTHEAST:
            self.commands.append(
                (self.pos[0] + settings.SPEED_FACTOR, self.pos[1] + settings.SPEED_FACTOR, self.pos[2], 'S'))
            self.commands.append(
                (self.pos[0] - settings.SPEED_FACTOR, self.pos[1] - settings.SPEED_FACTOR, self.pos[2], 'SV'))

        elif self.pos[2] == DIRECTION.SOUTHWEST:
            self.commands.append(
                (self.pos[0] - settings.SPEED_FACTOR, self.pos[1] - settings.SPEED_FACTOR, self.pos[2], 'S'))
            self.commands.append(
                (self.pos[0] + settings.SPEED_FACTOR, self.pos[1] + settings.SPEED_FACTOR, self.pos[2], 'SV'))

        elif self.pos[2] == DIRECTION.SOUTHEAST:
            self.commands.append(
                (self.pos[0] + settings.SPEED_FACTOR, self.pos[1] - settings.SPEED_FACTOR, self.pos[2], 'S'))
            self.commands.append(
                (self.pos[0] - settings.SPEED_FACTOR, self.pos[1] + settings.SPEED_FACTOR, self.pos[2], 'SV'))
