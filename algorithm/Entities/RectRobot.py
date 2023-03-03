import pygame


import algorithm.settings as settings


class RectRobot:
    def __init__(self, pos):
        self.pos = pos
        self.rect = pygame.Rect(pos, (6 * settings.BLOCK_SIZE, 6 * settings.BLOCK_SIZE))
        self.rect.bottomleft = self.pos

    def isCollided(self, obstacle):
        pos = (obstacle.pos[0] - 15, obstacle.pos[1] + 25)
        obRect = pygame.Rect(pos, (160, 160))
        obRect.topleft = pos
        return self.rect.colliderect(obRect)

