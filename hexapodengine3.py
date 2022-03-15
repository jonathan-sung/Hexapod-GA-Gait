import pybullet as p
import math
import numpy as np
import pybullet_data
import time
import bezier
import abc
import serial
import random


class Gait:
    def __init__(self, chromosome):
        self.duration = chromosome[0]
        self.legSequences = []
        for j in range(6):
            ls = LegSequence(j, chromosome)
            self.legSequences.append(ls)

    def evaluate(self, elapsed_time):
        leg_positions = []
        for ls in self.legSequences:
            leg_positions.append(ls.evaluate((elapsed_time % self.duration) / self.duration))
        return leg_positions


class LegSequence:
    def __init__(self, legID, chromosome):
        self.duration = chromosome[0]
        self.legID = legID
        self.motions = self.tripodMotions(legID, chromosome)
        self.motions.sort(key=lambda x: x[0])

    def tripodMotions(self, legID, chromosome):
        motions = []
        if legID % 2:
            motions.append((0.0, BezierMotion(getBezierPointsFromGenes(chromosome[10:]), getVelocityKnotsFromGenes(chromosome[6:10]))))
            motions.append((chromosome[1], LineMotion(chromosome[19:22], chromosome[10:13], getVelocityKnotsFromGenes(chromosome[2:6]))))
        else:
            motions.append((0.0, LineMotion(chromosome[19:22], chromosome[10:13], getVelocityKnotsFromGenes(chromosome[2:6]))))
            motions.append((chromosome[1], BezierMotion(getBezierPointsFromGenes(chromosome[10:]), getVelocityKnotsFromGenes(chromosome[6:10]))))
        return motions

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


def getBezierPointsFromGenes(genes):
    return np.array([[genes[0], genes[3], genes[6], genes[9]], [genes[1], genes[4], genes[7], genes[10]], [genes[2], genes[5], genes[8], genes[11]]])


def getVelocityKnotsFromGenes(genes):
    return np.array([[0.0, genes[0], genes[2], 1.0], [0.0, genes[1], genes[3], 1.0]])


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
    target_pos = []

    link_indexes = [x for x in range(3, 24, 4)]

    for j in range(6):
        rest_poses.append(pos_body + p.multiplyTransforms([0, 0, 0], orn_body, baseToEndEffectorConstVec[j], [0, 0, 0, 1])[0])
        tp = p.multiplyTransforms(rest_poses[j], orn_body, feet_positions[j], [0, 0, 0, 1])
        target_pos.append(tp[0])
    # ik2 = [0] * 18
    iks = []
    if not IK_ARRAY_CALC:
        for j in range(6):
            temp_ik = p.calculateInverseKinematics(
                hexapod_ID,
                link_indexes[j],
                target_pos[j]
            )
            temp_ik = list(temp_ik[(3 * j):3 + (3 * j)])
            # print(len(temp_ik), temp_ik)
            iks.extend(temp_ik)
        # print(iks)
        # iks = list(temp_ik[:3]) + [0] * 15
    else:
        iks = p.calculateInverseKinematics2(
            hexapod_ID,
            link_indexes,
            target_pos,
            lowerLimits=ll,
            upperLimits=ul,
            jointRanges=jr,
            restPoses=rest_poses
        )
        # print(ik, len(ik))
    return iks


def setServoStatesManual():
    p.setJointMotorControlArray(
        hexapod_ID,
        JOINT_INDICES,
        p.POSITION_CONTROL)
    # targetPositions=read_debug_parameters())


def setServoStatesLegs(feet_positions, force, jointDamping):
    tp = calculateIK3(feet_positions, jointDamping)
    p.setJointMotorControlArray(
        hexapod_ID,
        JOINT_INDICES,
        p.POSITION_CONTROL,
        targetPositions=tp,
        forces=([force] * 18)
    )


def plotCurve(bezier_curve):
    bezier_curve.plot(num_pts=256)


def flatten(t):
    return [item for sublist in t for item in sublist]


def angleBetweenVectors(a, b):
    unit_vector_1 = a / np.linalg.norm(a)
    unit_vector_2 = b / np.linalg.norm(b)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    return angle


def collidingLegs():
    # numOfCollisions = 0
    for j in range(24):
        aabb = (p.getAABB(hexapod_ID, j))
        familyOfLinks = [x for x in range(24) if math.floor(j / 4) == math.floor(x / 4)] + [-1]
        # collisionObjects = [x[1] for x in p.getOverlappingObjects(aabb[0], aabb[1]) if x[1] not in familyOfLinks and (j not in FEET_INDEXES or x[0] == hexapod_ID)]
        collisionObjects = [x[1] for x in p.getOverlappingObjects(aabb[0], aabb[1]) if (j not in FEET_INDEXES and (x[1] not in familyOfLinks or x[0] != hexapod_ID)) or (j in FEET_INDEXES and x[1] not in familyOfLinks and x[0] == hexapod_ID)]
        if len(collisionObjects) > 0:
            # print("hit", time.time())
            return True
    return False


def inverseCurve(x, a):
    return a / (a + (x * x))


def oldevaluateGait(individual):
    lastTime = time.time()
    while True:
        dt = 0
        gait = Gait(individual)
        resetEnvironment()
        for ii in range(3000):
            setServoStatesLegs(gait.evaluate(dt), individual[1], 0)
            p.stepSimulation()
            dt += 1. / 240.
            # time.sleep(1./240.)
        gaitEvaluation = 1 #gaitScore(hexapod_ID)
        # print(f'Time Elapsed: {time.time() - lastTime} seconds')
        print(f'PyBullet Instance ID: {UNIQUE_THREAD_ID} | Time Elapsed: {time.time() - lastTime} seconds | Evaluation: {gaitEvaluation} | Chromosome: {individual}')
        # print(f'Chromosome: {individual}')
        if not math.isnan(gaitEvaluation[0]):
            break
        else:
            print("PyBullet Glitch")
            resetPyBulletSimulation()
    return gaitEvaluation


def evaluateGait(individual):
    lastTime = time.time()
    numOfPhysicsSteps = 3000
    samplesPerEval = 100
    stabilityUpdateRate = int(numOfPhysicsSteps / samplesPerEval)
    stabilityScore = 0
    heightScore = 0
    collisionScore = 0
    gait = Gait(individual)
    while True:
        dt = 0
        resetEnvironment()
        for ii in range(numOfPhysicsSteps):
            if ii % stabilityUpdateRate == 0:
                hexapodBasePosAndOrn = p.getBasePositionAndOrientation(hexapod_ID)
                currentStability = sum([abs(angle) for angle in list(p.getEulerFromQuaternion(hexapodBasePosAndOrn[1]))])
                currentHeight = abs(TARGET_HEIGHT - hexapodBasePosAndOrn[0][2])
                stabilityScore += currentStability
                heightScore += currentHeight
                collisionScore += collidingLegs()
            setServoStatesLegs(gait.evaluate(dt), 150, 0)
            p.stepSimulation()
            dt += 1. / 240.
            # time.sleep(1. / 30.)
        hexapodBasePosAndOrn = p.getBasePositionAndOrientation(hexapod_ID)
        currentPosition = hexapodBasePosAndOrn[0]
        distance = hexapodBasePosAndOrn[0][1]
        straightness = abs(angleBetweenVectors(np.array([0, 1]), np.array([currentPosition[0], currentPosition[1]])))
        avgHeight = abs(heightScore / samplesPerEval)
        avgStability = stabilityScore / samplesPerEval
        avgNumOfCollisions = collisionScore / samplesPerEval
        fitness_distance = distance / 100.0
        fitness_straight = 1.0 - (straightness / math.pi)
        fitness_stability = inverseCurve(avgStability, 1)
        fitness_height = inverseCurve(avgHeight, 1)
        fitness_collisions = round(1 - avgNumOfCollisions, 2)
        fitness_total = (fitness_distance + fitness_straight + fitness_stability + fitness_height + fitness_collisions) / 5.0
        print(f'ID: {UNIQUE_THREAD_ID} | Time Elapsed: {time.time() - lastTime} | Evaluation: {fitness_distance, fitness_straight, fitness_stability, fitness_height, fitness_collisions, fitness_total} | Chromosome: {individual}')
        if not math.isnan(distance):
            break
        else:
            print("PyBullet Glitch")
            resetPyBulletSimulation()
    return fitness_total,


def runGait(individual):
    # ssc32 = serial.Serial('COM5', 115200, timeout=2)  # open serial port
    lastTime = time.time()
    gait = Gait(individual)
    resetEnvironment()
    p.setRealTimeSimulation(1)
    while True:
        setServoStatesLegs(gait.evaluate((time.time() - programStartTime) * 1), 150, 0)
        # updateRealServos(ssc32, 100)


def evaluateTest(individual):
    return 1, 1


def resetLegJoints():
    p.resetJointStatesMultiDof(hexapod_ID, JOINT_INDICES, [[0]] * 18, targetVelocities=[[0]] * 18)
    p.setJointMotorControlArray(hexapod_ID, JOINT_INDICES, p.POSITION_CONTROL, targetPositions=([0] * 18), forces=([150] * 18))


def resetEnvironment():
    resetLegJoints()
    p.resetBasePositionAndOrientation(hexapod_ID, [0, 0.01, 1.375 + random.uniform(0, 0.002)], [0, 0, 0, 1])
    p.stepSimulation()


def resetPyBulletSimulation():
    global plane_ID
    global hexapod_ID
    p.resetSimulation()
    p.setGravity(0, 0, -9.8)
    plane_ID = p.loadURDF("plane.urdf", globalScaling=4)
    hexapod_ID = p.loadURDF("robot3.urdf", [0, 0.01, 1.375 + random.uniform(0, 0.002)], [0, 0, 0, 1])


# start of main program
NUM_OF_LEGS = 6
UNIQUE_THREAD_ID = random.randint(1, 10000)
IK_ARRAY_CALC = True

# PyBullet Init
physicsClient = None
if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)
else:
    physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_ID = None
hexapod_ID = None
resetPyBulletSimulation()
programStartTime = time.time()
servoRangeOfMotion = (math.pi * 3 / 8)
FEET_INDEXES = [x for x in range(0, 24) if (x + 1) % 4 == 0]
TARGET_HEIGHT = 1.375
p.setRealTimeSimulation(0)

servoRangeOfMotion = (math.pi * 3 / 8)
ll = ([-servoRangeOfMotion] * 3) + ([0] * 15)
ul = ([servoRangeOfMotion] * 3) + ([0] * 15)
jr = ([servoRangeOfMotion] * 3) + ([0] * 15)
jd = ([0] * 18)

JOINT_INDICES = [x for x in range(0, 24) if (x + 1) % 4 != 0]

baseToEndEffectorConstVec = []
for i in range(3, 24, 4):
    baseToEndEffectorConstVec.append(np.array(p.getLinkState(hexapod_ID, i)[4]) - np.array(p.getBasePositionAndOrientation(hexapod_ID)[0]))

print(f'PyBullet Instance ID: {UNIQUE_THREAD_ID}')


def main():
    testChromosome = [2, 0.5] + ([0.5] * 8) + [0.0, -0.5, 0.0] + [0.0, 0.0, 0.6] + [0.0, 0.5, 0.6] + [0.0, 0.5, 0.0]
    # runGait(testChromosome)
    # evaluateGait(testChromosome)
    # runGait(manualChromosomeCreation())
    runGait([4.533811957914266, 0.22107850222064218, 0.42278529010140325, 0.7486144177960535, 0.18317199452838523, 0.4117632671373601, 0.7473699369065012, 0.8810782335963654, 0.45773127676238284, 0.10918880715771717, -0.13810874063881345, -0.4059553223323313, -0.2806492925654228, -0.08950159733529488, 0.7297791771963045, 0.9671788490214537, -0.09970797938255957, -1.5548401766416517, -1.196032093971841, 0.12416010205203688, 0.351810916832159, 1.1269481048958427])
    p.disconnect(physicsClient)


if __name__ == "__main__":
    main()
