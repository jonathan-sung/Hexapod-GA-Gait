import pygame

pygame.init()
joysticks = []

# for al the connected joysticks
for i in range(0, pygame.joystick.get_count()):
    # create a Joystick object in our list
    joysticks.append(pygame.joystick.Joystick(i))
    # initialize them all (-1 means loop forever)
    joysticks[-1].init()
    # print a statement telling what the name of the controller is
    print("Detected joystick"), joysticks[-1].get_name(), "'"

while True:
    for event in pygame.event.get():
        # The 0 button is the 'a' button, 1 is the 'b' button, 2 is the 'x' button, 3 is the 'y' button
        if event.button == 0:
            print("A Has Been Pressed")
