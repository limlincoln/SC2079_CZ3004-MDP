import pygame

import settings


class RectRobot:
    def __init__(self, pos):
        self.pos = pos
        self.rect = pygame.Rect(pos, (6 * settings.BLOCK_SIZE, 6 * settings.BLOCK_SIZE))

    def isCollided(self, obstacle):
        self.rect.bottomleft = self.pos
        return self.rect.colliderect(obstacle)

