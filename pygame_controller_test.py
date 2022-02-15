# Init game controller input
# pygame.init()
# pygame.joystick.init()
# joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
# joystick = joysticks[0]
#
# # Game controller input
#     pygame.event.pump()
#     left_stick = np.array([joystick.get_axis(0), joystick.get_axis(1)])
#     left_stick_magnitude = np.linalg.norm(left_stick)
#     if left_stick_magnitude > 0.5:
#         print(left_stick)
#
#         if abs(left_stick[1]) > 0.5:
#         counter += dt * -left_stick[1]
#         setServoStatesLegs(testGait.evaluate(counter))
#         # updateRealServos(ssc32, 150)

# p.resetBasePositionAndOrientation(hexapod_ID, [0, 0, 1.5], [0, 0, 0, 1])
#     for i in range(5000):
#         # print(distanceFromOrigin(hexapod_ID))
#         # # Update timing variables
#         # now = time.time()
#         # runTime = now - programStartTime
#         # dt = now - lastTime
#         # lastTime = now
#         counter += 1. / 240.
#
#         setServoStatesLegs(testGait.evaluate(counter))
#
#         # for i in range(6):
#         #     currentPos[i] = p.getLinkState(hexapod_ID, (4 * i) + 3)[0]
#         #     p.addUserDebugLine(prevPos[i], currentPos[i], [0, 0, 0.3], 4, gaitDuration)
#         #     prevPos[i] = currentPos[i]
#
#         p.stepSimulation()
#         # time.sleep(1./240.)
#     print(f'Time Elapsed: {time.time() - lastTime}')
#     print(f'Evaluation: {gaitScore(hexapod_ID)}')
