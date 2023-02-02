class Rectangle:
    """
    create a rectangle used for collision check
    """

    def __init__(self, pos, type):
        if type == 'R':
            self.x = pos[0]
            self.y = pos[1] + 30
            self.length = 30
        elif type == 'O':
            self.x = pos[0] - 15
            self.y = pos[0] + 25
            self.length = 40

    def isCollided(self, rec2):
        """
        Check if two rectangles are collided
        by checking on the following conditions:
        Is the RIGHT edge of r1 to the RIGHT of the LEFT edge of r2?
        Is the LEFT edge of r1 to the LEFT of the RIGHT edge of r2?
        Is the BOTTOM edge of r1 BELOW the TOP edge of r2?
        Is the TOP edge of r1 ABOVE the BOTTOM edge of r2?
        :param Rec2: Rectangle
        :return:
        bool value true if collided
        """

        if (self.x + self.length >= rec2.x and
                self.x <= rec2.x + rec2.length and
                self.y + self.length >= rec2.y and
                self.y <= rec2.y + rec2.length):
            return True
        return False
