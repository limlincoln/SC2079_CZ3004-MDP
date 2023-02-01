import math
import algorithm.settings

class Command:
    def __init__(self, time):
        self.time = time
        self.ticks = math.ceil(time * algorithm.settings.FRAMES)
        self.total_ticks = self.ticks

    def tick(self):
        self.ticks -= 1

    def doOneTick(self, robot):
        pass
