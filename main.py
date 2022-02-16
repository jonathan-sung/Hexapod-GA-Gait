import pybullet as p
import math
import numpy as np
import pybullet_data
import time
import bezier
import abc
import serial
import matplotlib.pyplot as plt
import pygame

from deap import base
from deap import creator
from deap import tools
import random
import seaborn as sns
import elitism


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
        for ii in range(MAX_MOTIONS_IN_SEQUENCE):
            local_index = ii * SIZE_OF_MOTION_CHROMOSOME
            motion_type = round(chromosome[0 + local_index])
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


def calculateIK3(feet_positions, jointDamping):
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
        jointDamping=([jointDamping] * 18)
    )
    return ik


def setServoStatesManual():
    p.setJointMotorControlArray(
        hexapod_ID,
        ([x for x in list(range(0, 24)) if x not in list(range(3, 24, 4))]),
        p.POSITION_CONTROL)
    # targetPositions=read_debug_parameters())


def setServoStatesLegs(feet_positions, force, jointDamping):
    p.setJointMotorControlArray(
        hexapod_ID,
        ([x for x in list(range(0, 24)) if x not in list(range(3, 24, 4))]),
        p.POSITION_CONTROL,
        targetPositions=calculateIK3(feet_positions, jointDamping),
        forces=([force] * 18)
    )


def plotCurve(bezier_curve):
    bezier_curve.plot(num_pts=256)


def flatten(t):
    return [item for sublist in t for item in sublist]


def manualChromosomeCreation():
    duration = 2
    force = 150
    jointDamping = 0
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
    # print(chromosome)
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
    lastTime = time.time()
    dt = 0
    gait = Gait(individual)
    resetLegJoints()
    p.resetBasePositionAndOrientation(hexapod_ID, [0, 0, 1.5], [0, 0, 0, 1])
    p.stepSimulation()
    for ii in range(3000):
        dt += 1. / 240.
        setServoStatesLegs(gait.evaluate(dt), gait.motorForce, gait.jointDamping)
        p.stepSimulation()
        # print(ii)
        # time.sleep(1. / 30.)
    gaitEvaluation = gaitScore(hexapod_ID)
    print(f'Time Elapsed: {time.time() - lastTime} seconds')
    print(f'Evaluation: {gaitScore(hexapod_ID)}')
    print(f'Chromosome: {individual}')
    return gaitEvaluation[0],


def runGait(individual):
    lastTime = time.time()
    gait = Gait(individual)
    resetLegJoints()
    p.resetBasePositionAndOrientation(hexapod_ID, [0, 0, 1.5], [0, 0, 0, 1])
    p.setRealTimeSimulation(1)
    for ii in range(100000):
        setServoStatesLegs(gait.evaluate(time.time() - programStartTime), gait.motorForce, gait.jointDamping)
        p.setRealTimeSimulation(1)

    gaitEvaluation = gaitScore(hexapod_ID)
    print(f'Time Elapsed: {time.time() - lastTime} seconds')
    print(f'Evaluation: {gaitScore(hexapod_ID)}')
    print(f'Chromosome: {individual}')
    return gaitEvaluation[0],


def resetLegJoints():
    for joint_index in [x for x in list(range(0, 24)) if x not in list(range(3, 24, 4))]:
        p.resetJointState(hexapod_ID, joint_index, 0)


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

# PySerial init
# ssc32 = serial.Serial('COM5', 115200, timeout=5)  # open serial port

# GENETIC ALGORITHM MAIN PROGRAM START
DIMENSIONS = 3 + (SIZE_OF_MOTION_CHROMOSOME * MAX_MOTIONS_IN_SEQUENCE * NUM_OF_LEGS)  # number of dimensions
BOUNDS_LOW = [0.1, 50, 0] + ((([0] * 6) + ([-2] * 12)) * 4 * 6)
BOUNDS_HIGH = [12, 300, 0] + (([4] + [1] + ([1] * 4) + ([2] * 12)) * 4 * 6)
POPULATION_SIZE = 50
P_CROSSOVER = 0.9  # probability for crossover
P_MUTATION = 0.5  # (try also 0.5) probability for mutating an individual
MAX_GENERATIONS = 10
HALL_OF_FAME_SIZE = 2
CROWDING_FACTOR = 20.0  # crowding factor for crossover and mutation
# RANDOM_SEED = 42
# random.seed(RANDOM_SEED)
toolbox = base.Toolbox()
creator.create("FitnessMax", base.Fitness, weights=(1.0,))
creator.create("Individual", list, fitness=creator.FitnessMax)

for i in range(DIMENSIONS):
    toolbox.register("layer_size_attribute_" + str(i), random.uniform, BOUNDS_LOW[i], BOUNDS_HIGH[i])

layer_size_attributes = ()
for i in range(DIMENSIONS):
    layer_size_attributes = layer_size_attributes + \
                            (toolbox.__getattribute__("layer_size_attribute_" + str(i)),)

toolbox.register("individualCreator", tools.initCycle, creator.Individual, layer_size_attributes, n=1)
toolbox.register("populationCreator", tools.initRepeat, list, toolbox.individualCreator)
toolbox.register("evaluate", evaluateGait)
toolbox.register("select", tools.selTournament, tournsize=2)
toolbox.register("mate", tools.cxSimulatedBinaryBounded, low=BOUNDS_LOW, up=BOUNDS_HIGH, eta=CROWDING_FACTOR)
toolbox.register("mutate", tools.mutPolynomialBounded, low=BOUNDS_LOW, up=BOUNDS_HIGH, eta=CROWDING_FACTOR, indpb=2.0 / 100.0)


# runGait([0.3929596408986611, 271.5139665796697, 0.02179784665106128, 1.8532968830347771, 0.8492462918927379, 0.554534014911483, 0.9096399675609805, 0.8631464614091577, 0.6489863769729364, -0.6770792952290772, -1.2024231662364053, -1.4226953255473451, 1.5657287040645171, 1.6053243429456732, -1.6189544370735007, 0.6260921241798696, -0.0980919955334536, 0.31259701595971506, 0.9099843049743708, -1.6152187666737388, 1.6704164849841168, 1.9912046865957305, 0.8293674927890938, 0.85144903837633, 0.685185310139843, 0.6545103596275429, 0.6079054070042292, 1.1102074278084582, 0.693180338561957, -0.6042208051945435, -0.1429678912184978, 1.3093902668780204, 0.2596078010020837, -1.0879051795955774, 1.347018982021124, 1.78843469217256, -0.7073890471863828, 0.3236779127527265, 0.3828557829603574, 2.252213853759304, 0.7013671641009545, 0.17332385418997887, 0.6435254405550995, 0.7997974932236792, 0.5231552215113654, 0.9598199597506072, -1.6473612477844062, -0.35485016116272605, -1.547468916231787, 1.1620927241868553, -0.5478712815764192, -1.1273764231357206, 0.3320558180508577, 1.721401666493682, 0.976388555863589, 1.2473911660001924, -1.6446123951381832, 2.021160098856527, 0.09718775551726816, 0.342321509565681, 0.8529443037095106, 0.4547096450553581, 0.10829293371628797, 1.6866469184200623, -0.8774585545705775, 1.6533657496898324, -0.1314966141139049, -0.4687640805871074, -0.12449864464414281, 0.4047860500294303, 0.1980606696116589, -1.9006493114912426, -0.053924610714671584, 0.057224826109606663, -1.6053992664376846, 2.3444318274374107, 0.03903633256930199, 0.1833346391599111, 0.7362259000311269, 0.34008815606961723, 0.784035644564481, -0.14384890113589843, -0.9144711237507601, -0.7199610080510607, -1.069476235764114, 1.2804900985689844, 1.5159115631098559, -1.2922936539054901, -0.25878588389995005, 1.2721950360813858, 1.7535510141358024, -0.3825454048992485, 0.3433487504761775, 0.23580375481731908, 0.22751392580364121, 0.6407101139434426, 0.5525495045034661, 0.9183703476020562, 0.6027091138539926, -0.4146489413542124, -1.4959212145427974, -1.6996182449739472, 0.6873943613705121, -1.0211800013055639, 0.20728570130698842, -1.1309601490370387, -1.4650249819934855, 0.38757094348841814, 1.941472984007417, -1.92075169833105, 0.3096858785793627, 2.6462405817117554, 0.9466329014243898, 0.903643718935992, 0.6773134123483033, 0.3811245612529034, 0.12675146346463406, -1.2448182806759776, 1.2096950959181225, 0.5113006133444072, -0.1595925161915645, 0.4003195762924433, -1.7631801159916058, -0.001974859232783621, -0.11568610269787594, -0.7093572453746229, 0.5774012333639004, -0.8391696865875383, 1.017665452010056, 1.4473452121283188, 0.9202466508303866, 0.9536566309588177, 0.9664047348193652, 0.008340561840553429, 0.414557747893876, -1.615962731390875, -0.9615336788676241, -1.1342411575439386, -1.0700100378285649, 1.602416940477214, 1.624120102319282, 1.7518527501908585, 0.2824915276107984, 1.818405036333659, 1.2390320920728766, -0.1539677572471415, 0.45802462406491806, 0.2475461448573706, 0.43629885406398805, 0.5062440885751975, 0.7825837031373583, 0.8442766436188887, 0.9157499757058284, 0.3255211548184844, 1.228161858640401, 0.8297688863125858, 1.5227117309284082, -1.2543146836505394, -1.1167265202219159, 0.47909607697266604, 1.307869100384197, -1.0971817030582358, 0.7482676035545444, 0.671932249611473, 1.7729935231123681, 3.608296250343415, 0.952381984331194, 0.2932667880088783, 0.5309172641119879, 0.55448540993255, 0.38533252365175774, 1.4661541548932577, 1.2489349067127273, -0.7467690121128916, -0.40027519479373797, 1.3243321900767535, -0.6723634093773918, 1.9814534805633683, 1.7409267900795382, 0.8869268636428124, 1.5501896463732656, -1.2238882789583938, -0.44070502211271656, 2.1920493218413974, 0.9922850450201985, 0.9822075695010913, 0.04266402383640954, 0.9377092756637283, 0.8217392315248615, -0.6477506438838935, 0.055280003442610526, -1.6905709010231098, -1.2205038058814357, 1.4518107831562812, -1.6861948132741114, -0.6152754461688411, -0.3439722733987532, -1.8988477066435918, 0.6547229650311512, -1.3423875521358304, -0.6932045874706789, 1.269712966268086, 0.8057572340331309, 0.4439861727164672, 0.579184054886488, 0.06097794021019726, 0.5650574090541826, -0.3671711294104478, -0.07430270211761256, 1.3127926184109424, 1.7231470720469317, 0.10705487627913679, -1.0480030946666614, 0.6738173967857991, -0.29320656893164704, 0.7507463678674542, -0.9187410127951625, 1.4621875567553182, 1.978293825827143, 1.633043242687323, 0.5238715280009296, 0.5510864512407156, 0.6298485398823238, 0.12329690693412618, 0.3574057066001546, 1.5121851034858746, -0.35322969925409237, 1.301202634262455, 0.9867591550353965, 0.6115930296640109, -0.3963092535140088, 1.0547972961115355, 1.6474974632182078, -1.340427209143473, -1.3678032685907149, 1.3359237469579208, 1.059120759571917, 0.2478740111553388, 0.18617914200794616, 0.34774147385197063, 0.9348721647034101, 0.022999636044332375, 0.5440694273652704, 1.142217272891032, -1.8268417232099434, 0.031158754270179778, 0.8557715692686951, 1.0620473277031184, 0.000781513920318111, 0.3774494776234629, 0.6883462770678597, 0.8553326752447223, -0.367292638210583, -1.6457276932902936, 0.46482514644250394, 3.3183311964115507, 0.3234406510872574, 0.6441937462588783, 0.71620964712244, 0.5536819957194653, 0.8834587614147069, -0.12673080251551472, -0.3130162857562026, -0.46818509737169034, 0.8949755307460807, -1.2078289897367571, -0.895549918287196, -0.8799608578311111, -1.9623666496328824, 1.895533284630842, -1.4597254841407006, 1.4872971753141186, -0.9264001515965674, 0.305867274285466, 0.2179968955646554, 0.09087440155108728, 0.38313039487973344, 0.7142409603228854, 0.5450618601973616, -0.849398771599963, -1.5604475327236265, 1.4356067373248453, 1.6578365644937574, 0.755073710441031, 0.8868534941748853, -0.9942927627329854, 1.6976773925601096, -1.4700834983422162, 1.8836021413427277, -0.4188204440877618, 1.5543520521783374, 1.3620656907557127, 0.001409600301766012, 0.487037262545826, 0.5822703183642449, 0.7929970789537821, 0.12475752635864557, 0.06587321106771649, 1.0089069399029411, 0.7638243102515021, 0.9224877032551977, -0.7319715118847039, -1.1456269197609994, -0.08415890459020081, -1.5198429485705252, 1.2059669990465576, 0.35595148266461263, 0.09613387285368162, 1.7447344493332273, 1.5862874715237112, 0.10054264234647436, 0.2541306357086999, 0.2334000566335393, 0.2117136127542236, 0.7303932244444657, -0.8245185043747635, -0.44842004243410694, -0.924383430130398, 1.4272601121994766, 1.6099170062995323, -0.7460859809757812, -0.4852596340179135, -1.2359970483211054, -1.148441654857184, 1.9995408770178726, -0.37980152788637084, 1.374234934177405, 2.09017258141923, 0.7369367400533136, 0.9098831729429618, 0.1460775580432107, 0.5544035221523244, 0.4745493003837572, -0.6368630843743155, -0.5513448018168514, 0.0750392707095453, 0.7940426936327051, -0.7676076770396696, 0.5104991209663619, -1.6304023802706158, -1.2916141820794969, 1.0711892367943634, 0.3898807116562596, 1.6922124127961888, -0.13864290425980405, 2.967551595489912, 0.3307299758510207, 0.30941510168215935, 0.598195505864068, 0.5646352813873388, 0.852973608530323, -0.463631446810841, -0.944595308418752, -1.3125606053813108, -0.10123363438610405, -1.9409118349571859, 1.6453736989618446, -1.7713506607149667, 0.0020045904273217063, -1.818144202073888, -1.2541158356297104, 0.4299744064157397, 1.8434868688859585, 3.8180546251873335, 0.6093106416224519, 0.9644979988287212, 0.8602117694676918, 0.8123880835284422, 0.6842880973295431, -0.13954566121656675, -1.8167642927060814, 0.8160045767363266, -1.6346102891436372, -1.6352284399388584, -1.4635869399233195, -1.951822728521103, 0.6546852340806124, -1.1111091587792277, 1.2652769269100803, -1.7116500890281419, 1.3447345251930551, 2.4384087309721534, 0.7207894027245089, 0.13732216695963748, 0.9790471956095228, 0.8878870782294243, 0.7362554825070255, 0.6142832916290202, -0.25260991833107327, 0.7332820363129144, -1.7923870552386652, 1.127810613771877, 0.13424348815955411, 1.1637015509111794, -1.548506570304124, -1.3250801203618767, -0.3996992285618115, -1.062571699252071, 1.7818023065305537, 1.2229067952273753, 0.2559430728821008, 0.6446833295504868, 0.4955503284206849, 0.6613993965651008, 0.10098293246158996, 1.0603371211112682, -0.364910936751766, 0.5510804747786805, 1.7817029673797768, 1.1083710283746402, -0.03838355352174139, 0.29480572186152687, 0.9164264658960328, 0.384329151282344, -0.053652679418499424, -0.5227425488481761, -0.9535315704072886, 2.1197526630618477, 0.8865058803694138, 0.6770665352299008, 0.7586376652838565, 0.9191708493671651, 0.1632821757662415, 1.2741038247130931, 1.6943840823275909, -1.321038432867731, 0.1181301473346546, -1.7809059537385103, -0.7291821244963348, -0.6413524267992272, 0.235948423525494, 1.5634383061782673, -1.6810692186976162, -1.205659319594056, -1.5492788934625252])
# Genetic Algorithm flow:
def main():
    population = toolbox.populationCreator(n=POPULATION_SIZE)
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("max", np.max)
    stats.register("avg", np.mean)
    hof = tools.HallOfFame(HALL_OF_FAME_SIZE)
    population, logbook = elitism.eaSimpleWithElitism(population, toolbox, cxpb=P_CROSSOVER, mutpb=P_MUTATION, ngen=MAX_GENERATIONS, stats=stats, halloffame=hof, verbose=True)
    best = hof.items[0]
    print("-- Best Individual = ", best)
    print("-- Best Fitness = ", best.fitness.values[0])
    maxFitnessValues, meanFitnessValues = logbook.select("max", "avg")

    # plot statistics:
    sns.set_style("whitegrid")
    plt.plot(maxFitnessValues, color='red')
    plt.plot(meanFitnessValues, color='green')
    plt.xlabel('Generation')
    plt.ylabel('Max / Average Fitness')
    plt.title('Max and Average fitness over Generations')
    plt.show()


if __name__ == "__main__":
    main()

# ssc32.close()  # close port
p.disconnect(physicsClient)
