import sys
import pygame
import settings
from Entities.arena import Arena
from Entities.Obstacle import Obstacle
from Entities.Robot import Robot

BLACK = (0, 0, 0)
WHITE = (200, 200, 200)
GREEN = (122,213,1)


def main():
    global SCREEN, CLOCK
    pygame.init()
    SCREEN = pygame.display.set_mode((settings.WINDOW_WIDTH, settings.WINDOW_HEIGHT))
    CLOCK = pygame.time.Clock()
    SCREEN.fill(BLACK)
    test = [Obstacle((80,80), "left", (4*settings.BLOCK_SIZE,4*settings.BLOCK_SIZE)), Obstacle((200,200), "top", (4*settings.BLOCK_SIZE,4*settings.BLOCK_SIZE))]
    arena = Arena(test, 800, 800, settings.BLOCK_SIZE)
    robot = Robot()

    while True:
        arena.drawGrid(SCREEN,WHITE,GREEN)
        robot.drawCar(SCREEN)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        pygame.display.update()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
