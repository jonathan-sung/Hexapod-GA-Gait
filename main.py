import pybullet as p
import math
import numpy as np
import pybullet_data
import time
import bezier
import abc
import serial
import matplotlib.pyplot as plt
import itertools
import pygame
import numpy.linalg as la


# roslaunch urdf_tutorial display.launch model:='D:\Programming\Python\Hexapod-GA-Gait\robot.urdf'
# roslaunch urdf_tutorial display.launch model:='D:\Programming\Python\Hexapod-GA-Gait\crab_description\models\crab_model.urdf'


class Gait:

    def __init__(self, chromosome):
        self.duration = chromosome[0]
        self.motorForce = chromosome[1]
        self.jointDamping = chromosome[2]
        self.motion_chromosome = chromosome[3:]
        self.legSequences = []
        for j in range(6):
            motions = self.motion_chromosome[j * SIZE_OF_MOTION_CHROMOSOME * MAX_MOTIONS_IN_SEQUENCE: (j + 1) * SIZE_OF_MOTION_CHROMOSOME * MAX_MOTIONS_IN_SEQUENCE]
            ls = LegSequence(chromosome[0], j, motions)
            self.legSequences.append(ls)

    def evaluate(self, elapsed_time):
        leg_positions = []
        for ls in self.legSequences:
            leg_positions.append(ls.evaluate((elapsed_time % self.duration) / self.duration))
        return leg_positions


class LegSequence:
    def __init__(self, duration, legID, motion_chromosome):
        self.duration = duration
        self.legID = legID
        # self.motions = self.manualInitMotions(legID)
        self.motions = self.readChromosome(motion_chromosome)
        self.motions.sort(key=lambda x: x[0])
        self.addNoMotionConsts()

    def readChromosome(self, chromosome):
        motions = []
        for i in range(MAX_MOTIONS_IN_SEQUENCE):
            local_index = i * SIZE_OF_MOTION_CHROMOSOME
            motion_type = chromosome[0 + local_index]
            start_time = chromosome[1 + local_index]
            if motion_type == 0:
                continue
            elif motion_type == 1:  # line motions
                motions.append((start_time, LineMotion([chromosome[6 + local_index], chromosome[7 + local_index], chromosome[8 + local_index]],
                                                       [chromosome[15 + local_index], chromosome[16 + local_index], chromosome[17 + local_index]],
                                                       np.array([[0.0, chromosome[2 + local_index], chromosome[4 + local_index], 1.0],
                                                                 [0.0, chromosome[3 + local_index], chromosome[5 + local_index], 1.0]]))))
            elif motion_type == 2:  # curve motions
                motions.append((start_time,
                                BezierMotion(np.array([[chromosome[6 + local_index], chromosome[9 + local_index], chromosome[12 + local_index], chromosome[15 + local_index]],
                                                       [chromosome[7 + local_index], chromosome[10 + local_index], chromosome[13 + local_index], chromosome[16 + local_index]],
                                                       [chromosome[8 + local_index], chromosome[11 + local_index], chromosome[14 + local_index], chromosome[17 + local_index]]]),
                                             np.array([[0.0, chromosome[2 + local_index], chromosome[4 + local_index], 1.0], [0.0, chromosome[3 + local_index], chromosome[5 + local_index], 1.0]]))))
            elif motion_type == 4:
                motions.append((start_time, NoMotion()))
        return motions

    def manualInitMotions(self, legID):
        startPos = [0.0, -0.5, 0.0]
        knot1 = [0.0, 0.0, 0.6]
        knot2 = [0.0, 0.5, 0.6]
        endPos = [0.0, 0.5, 0.0]
        motions = []
        if legID % 2:
            motions.append((0.0, BezierMotion(np.array([[startPos[0], knot1[0], knot2[0], endPos[0]], [startPos[1], knot1[1], knot2[1], endPos[1]], [startPos[2], knot1[2], knot2[2], endPos[2]]]))))
            motions.append((0.5, LineMotion(endPos, startPos)))
        else:
            motions.append((0.0, LineMotion(endPos, startPos)))
            motions.append((0.5, BezierMotion(np.array([[startPos[0], knot1[0], knot2[0], endPos[0]], [startPos[1], knot1[1], knot2[1], endPos[1]], [startPos[2], knot1[2], knot2[2], endPos[2]]]))))
        return motions

    def addNoMotionConsts(self):
        if len(self.motions) > 1:
            for j in range(len(self.motions)):
                if type(self.motions[j][1]) is NoMotion:
                    self.motions[j][1].setEvaluationConst(self.motions[(j - 1) % (len(self.motions))][1].evaluate(1.0))

    def getPositionInMotion(self, progress, motions):
        # for-loop to loop around every motion to return motion for a given amount of progress
        for j in range(len(motions) - 1, -1, -1):
            # print(f'j: {j}')
            motion_start_time = motions[j][0]
            end_time = motions[(j + 1) % (len(motions))][0]
            loop_back = (motion_start_time >= end_time)
            if progress < end_time and loop_back:
                progress += 1
            end_time += loop_back  # end of cycle motion loop back around
            if motion_start_time <= progress < end_time:
                # Convert global progress into local-progress of given motion
                local_progress = (progress - motion_start_time) / (end_time - motion_start_time)
                return motions[j][1].evaluate(local_progress)

        return [0.0, 0.0, 0.0]

    def evaluate(self, progress):
        return self.getPositionInMotion(progress, self.motions)


class Motion(metaclass=abc.ABCMeta):
    @abc.abstractmethod
    def evaluate(self, progress):
        pass


class BezierMotion(Motion):
    def __init__(self, knots, velocity_knots):
        self.knots = knots
        self.curve = bezier.Curve(self.knots, degree=3)
        self.velocity_curve = bezier.Curve(velocity_knots, degree=3)
        # plotCurve(self.velocity_curve)

    def evaluate(self, progress):
        # velocity curve: y=a+\frac{b}{\left(sx+1\right)^{7}}
        # progress = 1 + -(1 / math.pow((progress + 1), 7))
        curved_progress = self.velocity_curve.evaluate(progress)
        raw_position = self.curve.evaluate(curved_progress[1][0] % 1)
        position = flatten(raw_position)
        return position


class LineMotion(Motion):
    def __init__(self, startPoint, endPoint, velocity_knots):
        self.startPoint = startPoint
        self.endPoint = endPoint
        self.velocity_curve = bezier.Curve(velocity_knots, degree=3)
        # plotCurve(self.velocity_curve)

    # Parametric equation of a straight line given a progression percentage
    def evaluate(self, progress):
        # velocity curve: y=a+\frac{b}{\left(sx+1\right)^{7}}
        # progress = 1 + -(1 / math.pow((progress + 1), 7))

        curved_progress = self.velocity_curve.evaluate(progress % 1)
        x = self.startPoint[0] + (self.endPoint[0] - self.startPoint[0]) * curved_progress[1][0]
        y = self.startPoint[1] + (self.endPoint[1] - self.startPoint[1]) * curved_progress[1][0]
        z = self.startPoint[2] + (self.endPoint[2] - self.startPoint[2]) * curved_progress[1][0]
        return [x, y, z]


class NoMotion(Motion):
    def __init__(self, *args):
        self.evaluationConst = [0.0, 0.0, 0.0]
        if len(args) == 1:
            self.evaluationConst = args[0]

    def setEvaluationConst(self, c):
        self.evaluationConst = c

    def evaluate(self, progress):
        return self.evaluationConst


def radToPwm(angle):
    return ((2000 * angle) / math.pi) + 1500


# t in ms; the closer t is to 0, more accuracy but less smooth motion
def updateRealServos(ser, t):
    # right legs
    ser.write(
        f'#0P{radToPwm(-p.getJointState(hexapod_ID, 8)[0])}T{t}#1P{radToPwm(p.getJointState(hexapod_ID, 9)[0])}T{t}#2P{radToPwm(-p.getJointState(hexapod_ID, 10)[0])}T{t}\r'.encode(
            'utf-8'))
    ser.write(
        f'#4P{radToPwm(-p.getJointState(hexapod_ID, 4)[0])}T{t}#5P{radToPwm(p.getJointState(hexapod_ID, 5)[0])}T{t}#6P{radToPwm(-p.getJointState(hexapod_ID, 6)[0])}T{t}\r'.encode(
            'utf-8'))
    ser.write(
        f'#8P{radToPwm(-p.getJointState(hexapod_ID, 0)[0])}T{t}#9P{radToPwm(p.getJointState(hexapod_ID, 1)[0])}T{t}#10P{radToPwm(-p.getJointState(hexapod_ID, 2)[0])}T{t}\r'.encode(
            'utf-8'))

    # left legs
    ser.write(
        f'#24P{radToPwm(-p.getJointState(hexapod_ID, 12)[0])}T{t}#25P{radToPwm(p.getJointState(hexapod_ID, 13)[0])}T{t}#26P{radToPwm(-p.getJointState(hexapod_ID, 14)[0])}T{t}\r'.encode(
            'utf-8'))
    ser.write(
        f'#20P{radToPwm(-p.getJointState(hexapod_ID, 16)[0])}T{t}#21P{radToPwm(p.getJointState(hexapod_ID, 17)[0])}T{t}#22P{radToPwm(-p.getJointState(hexapod_ID, 18)[0])}T{t}\r'.encode(
            'utf-8'))
    ser.write(
        f'#16P{radToPwm(-p.getJointState(hexapod_ID, 20)[0])}T{t}#17P{radToPwm(p.getJointState(hexapod_ID, 21)[0])}T{t}#18P{radToPwm(-p.getJointState(hexapod_ID, 22)[0])}T{t}\r'.encode(
            'utf-8'))


def calculateIK3(feet_positions):
    pos_body = np.array(p.getBasePositionAndOrientation(hexapod_ID)[0])
    orn_body = p.getBasePositionAndOrientation(hexapod_ID)[1]

    rest_poses = []
    local_target_pos = []
    translated_pos = []
    target_pos = []

    for j in range(6):
        rest_poses.append(pos_body + p.multiplyTransforms([0, 0, 0], orn_body, baseToEndEffectorConstVec[j], [0, 0, 0, 1])[0])
        local_target_pos.append(feet_positions[j])
        translated_pos.append(p.multiplyTransforms(rest_poses[j], orn_body, local_target_pos[j], [0, 0, 0, 1]))
        target_pos.append(translated_pos[j][0])

    ik = p.calculateInverseKinematics2(
        hexapod_ID,
        ([x for x in range(3, 24, 4)]),
        target_pos,
        solver=p.IK_DLS,
        lowerLimits=ll,
        upperLimits=ul,
        jointRanges=jr,
        restPoses=rest_poses,
        jointDamping=jd
    )
    return ik


def setServoStatesManual():
    p.setJointMotorControlArray(
        hexapod_ID,
        ([x for x in list(range(0, 24)) if x not in list(range(3, 24, 4))]),
        p.POSITION_CONTROL)
    # targetPositions=read_debug_parameters())


def setServoStatesLegs(feet_positions):
    p.setJointMotorControlArray(
        hexapod_ID,
        ([x for x in list(range(0, 24)) if x not in list(range(3, 24, 4))]),
        p.POSITION_CONTROL,
        targetPositions=calculateIK3(feet_positions),
        forces=([150] * 18)
    )


def plotCurve(bezier_curve):
    bezier_curve.plot(num_pts=256)


def flatten(t):
    return [item for sublist in t for item in sublist]


def manualChromosomeCreation():
    duration = 2
    force = 150
    jointDamping = 1
    chromosome = [duration, force, jointDamping]
    bezierMotion = [2, 0.0, 0.5, 0.5, 0.5, 0.5, 0.0, -0.5, 0.0, 0.0, 0.0, 0.6, 0.0, 0.5, 0.6, 0.0, 0.5, 0.0]
    lineMotion = [1, 0.5, 1.0, 0.0, 1.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0]
    for leg_id in range(NUM_OF_LEGS):
        if leg_id % 2 == 0:
            bezierMotion[1] = 0.0
            lineMotion[1] = 0.5
        else:
            bezierMotion[1] = 0.5
            lineMotion[1] = 0.0
        chromosome.extend(bezierMotion)
        chromosome.extend(lineMotion)
        for motion_id in range(MAX_MOTIONS_IN_SEQUENCE - 2):
            chromosome.extend([0] * SIZE_OF_MOTION_CHROMOSOME)
    return chromosome


def gaitScore(bodyID):
    current_position = p.getBasePositionAndOrientation(bodyID)[0]
    distance = distanceFromOrigin(bodyID)
    angle = angleBetweenVectors(np.array([0, 1]), np.array([current_position[0], current_position[1]]))
    return distance, abs(angle)


def distanceFromOrigin(bodyID):
    return p.getBasePositionAndOrientation(bodyID)[0][1]


def angleBetweenVectors(a, b):
    unit_vector_1 = a / np.linalg.norm(a)
    unit_vector_2 = b / np.linalg.norm(b)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    return angle


def evaluateGait(individual):
    dt = 0
    gait = Gait(individual)
    p.resetBasePositionAndOrientation(hexapod_ID, [0, 0, 1.5], [0, 0, 0, 1])
    for ii in range(5000):
        dt += 1. / 240.
        setServoStatesLegs(gait.evaluate(dt))
        p.stepSimulation()
    print(f'Time Elapsed: {time.time() - lastTime}')
    print(f'Evaluation: {gaitScore(hexapod_ID)}')


# start of main program
MAX_MOTIONS_IN_SEQUENCE = 4
NUM_OF_LEGS = 6
SIZE_OF_MOTION_CHROMOSOME = 18

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
hexapod_ID = p.loadURDF("robot2.urdf", [0, 0, 1.5], [0, 0, 0, 1])

servoRangeOfMotion = math.pi * 3 / 4

# setup IK parameters
ll = ([-servoRangeOfMotion] * 3) + ([0] * 15)  # lowerLimit
ul = ([servoRangeOfMotion] * 3) + ([0] * 15)  # upperLimit
jr = ([servoRangeOfMotion] * 3) + ([0] * 15)  # jointRange
jd = ([0] * 18)  # jointDamping

baseToEndEffectorConstVec = []
for i in range(3, 24, 4):
    baseToEndEffectorConstVec.append(
        np.array(p.getLinkState(hexapod_ID, i)[4]) - np.array(p.getBasePositionAndOrientation(hexapod_ID)[0]))

prevPos = []
currentPos = [0, 0, 0, 0, 0, 0]
for i in range(6):
    prevPos.append(p.getLinkState(hexapod_ID, (4 * i) + 3)[0])

programStartTime = time.time()
lastTime = programStartTime
counter = 0

# PySerial init
# ssc32 = serial.Serial('COM5', 115200, timeout=5)  # open serial port

# plt.show()
for j in range(100):
    evaluateGait(manualChromosomeCreation())

# ssc32.close()  # close port
p.disconnect(physicsClient)
