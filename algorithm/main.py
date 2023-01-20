import sys
import pygame
import settings
from Entities.arena import Arena
from Entities.Obstacle import Obstacle
from Entities.Robot import Robot



def main():
    global SCREEN, CLOCK
    pygame.init()
    SCREEN = pygame.display.set_mode((settings.WINDOW_WIDTH, settings.WINDOW_HEIGHT))
    CLOCK = pygame.time.Clock()
    SCREEN.fill(settings.BLACK)
    test = [Obstacle((80,80), "left", (2*settings.BLOCK_SIZE,2*settings.BLOCK_SIZE)), Obstacle((200,200), "top", (2*settings.BLOCK_SIZE,2*settings.BLOCK_SIZE))]
    arena = Arena(test, 800, 800, settings.BLOCK_SIZE)
    robot = Robot()
    arena.drawGrid(SCREEN)
    robot.drawCar(SCREEN)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    robot.moveStraight(10)
                elif event.key == pygame.K_d:
                    robot.turnRight(10)
                elif event.key == pygame.K_a:
                    robot.turnLeft(10)
                arena.updateGrid(robot, SCREEN)
        pygame.display.update()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
