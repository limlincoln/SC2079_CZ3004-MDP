import math

import settings


class STMCommand:
    def __init__(self, command):
        self.command = command
        self.tick = 0

    def setTick(self):
        if self.command == 's' or self.command  == 'b' or self.command == 'P':
            self.tick = math.ceil(0.75)
        elif self.command == 'd' or self.command == 'u':
            self.tick = math.ceil(3.95)
        elif self.command == 'v' or self.command == 'w':
            self.tick = math.ceil(4)
        elif self.command == 'OL' or self.command == 'OR':
            self.tick = math.ceil(8)
        return self.tick

    def yoloTick(self):
        self.tick -= 1
