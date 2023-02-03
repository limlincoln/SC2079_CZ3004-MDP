from algorithm.constants import DIRECTION


class Command:
    def __init__(self, pos):
        self.pos = pos
        self.commands = []
        self.dirList = [DIRECTION.TOP, DIRECTION.RIGHT, DIRECTION.BOTTOM, DIRECTION.LEFT]

    def getCommands(self):
        self.moveStraight()
        self.moveLeft()
        self.moveRight()
        return self.commands

    def moveStraight(self):
        """
        Move front and back
        :return:
        new pos forward and backward
        """
        if self.pos[2] == DIRECTION.TOP:
            self.commands.append((self.pos[0], self.pos[1] + 10, self.pos[2], 'S'))
            self.commands.append((self.pos[0], self.pos[1] - 10, self.pos[2], 'RV'))
        elif self.pos[2] == DIRECTION.LEFT:
            self.commands.append((self.pos[0] - 10, self.pos[1], self.pos[2], 'S'))
            self.commands.append((self.pos[0] + 10, self.pos[1], self.pos[2], 'RV'))
        elif self.pos[2] == DIRECTION.RIGHT:
            self.commands.append((self.pos[0] + 10, self.pos[1], self.pos[2], 'S'))
            self.commands.append((self.pos[0] - 10, self.pos[1], self.pos[2], 'SV'))
        else:
            self.commands.append((self.pos[0], self.pos[1] - 10, self.pos[2], 'S'))
            self.commands.append((self.pos[0], self.pos[1] + 10, self.pos[2], 'SV'))

    def moveRight(self):
        """
        turn right 90* (for now)
        :return:
        new pos after moving right
        """
        if self.pos[2] == DIRECTION.TOP:
            self.commands.append((self.pos[0], self.pos[1], self.dirList[(self.dirList.index(self.pos[2])+1) % 4], 'R'))
        elif self.pos[2] == DIRECTION.LEFT:
            self.commands.append((self.pos[0], self.pos[1], self.dirList[(self.dirList.index(self.pos[2]) + 1) % 4], 'R'))
        elif self.pos[2] == DIRECTION.RIGHT:
            self.commands.append((self.pos[0], self.pos[1], self.dirList[(self.dirList.index(self.pos[2]) + 1) % 4], 'R'))
        else:
            self.commands.append((self.pos[0], self.pos[1], self.dirList[(self.dirList.index(self.pos[2]) + 1) % 4], 'R'))



    def moveLeft(self):
        """
        turn left 90* (for now)
        :return:
        new pos after moving right
        """
        if self.pos[2] == DIRECTION.TOP:
            self.commands.append(
                (self.pos[0], self.pos[1], self.dirList[abs(self.dirList.index(self.pos[2]) - 1) % 4],
                 'L'))
        elif self.pos[2] == DIRECTION.LEFT:
            self.commands.append(
                (self.pos[0], self.pos[1], self.dirList[abs(self.dirList.index(self.pos[2]) - 1) % 4],
                 'L'))
        elif self.pos[2] == DIRECTION.RIGHT:
            self.commands.append(
                (self.pos[0], self.pos[1], self.dirList[abs(self.dirList.index(self.pos[2]) - 1) % 4],
                 'L'))
        else:
            self.commands.append(
                (self.pos[0], self.pos[1], self.dirList[abs(self.dirList.index(self.pos[2]) - 1) % 4],
                 'L'))





