import math

import serial


def centreAllLegs(ser):
    servoIDs_R = ([x for x in list(range(0, 12)) if x not in list(range(3, 24, 4))])
    servoIDs_L = ([x for x in list(range(16, 27)) if x not in list(range(3, 24, 4))])
    servoIDs = servoIDs_R + servoIDs_L
    print(servoIDs)
    for i in servoIDs:
        command = f'#{i}P1500\r'.encode('utf-8')
        ser.write(command)


def curlAllLegs(ser):
    servoIDs_R = [1, 2, 5, 6, 9, 10]
    servoIDs_L = [17, 18, 21, 22, 25, 26]
    servoIDs_M = [0, 4, 8, 16, 20, 24]
    for i in servoIDs_R:
        command = f'#{i}P750\r'.encode('utf-8')
        ser.write(command)
    for i in servoIDs_L:
        command = f'#{i}P2250\r'.encode('utf-8')
        ser.write(command)
    for i in servoIDs_M:
        command = f'#{i}P1500\r'.encode('utf-8')
        ser.write(command)

def radToPwm(angle):
    return int(((2000 * angle) / math.pi) + 1500)


ssc32 = serial.Serial('COM3', 115200, timeout=2)  # open serial port
#centreAllLegs(ssc32)
curlAllLegs(ssc32)
ssc32.close()  # close port
