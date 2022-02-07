import pygame

pygame.init()
pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
joystick = joysticks[0]

while True:
    pygame.event.pump()
    print(joystick.get_axis(0), joystick.get_axis(1))
