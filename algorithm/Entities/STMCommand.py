class STMCommand:
    def __init__(self, pos, command):
        self.command = command
        self.pos = pos
        self.tick = None

    def setTick(self):
        if self.command == 'S' or self.command  == 'SV':
            self.tick = 1
        elif self.command == 'L' or self.command == 'R':
            self.tick = 5
        elif self.command  == 'RR' or self.command == 'RL':
            self.tick  = 5
        elif self.command == 'OL' or self.command == 'OR':
            self.tick = 10
