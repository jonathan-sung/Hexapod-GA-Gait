import pybullet as p
import math
import pybullet_data
import time
import random
import numpy as np
import serial


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


def init_debug_parameters():
    for j in list(range(0, 6)):
        control_IDs.append(p.addUserDebugParameter(f"Pelvis {j}", -servoRangeOfMotion, servoRangeOfMotion, 0))
        control_IDs.append(p.addUserDebugParameter(f"Hip {j}", -servoRangeOfMotion, servoRangeOfMotion, 0))
        control_IDs.append(p.addUserDebugParameter(f"Knee {j}", -servoRangeOfMotion, servoRangeOfMotion, 0))


def read_debug_parameters():
    angles = []
    for x in control_IDs:
        angles.append(p.readUserDebugParameter(x))
    return angles


def chromosomeCreator():
    pos = []
    duration = 1
    force = 200
    # pos.extend([duration] + [0] * NUM_OF_SERVOS)
    for j in range(LENGTH_OF_SEQUENCE - 1):
        gaitState = [0] * NUM_OF_SERVOS
        if j < NUM_OF_SERVOS:
            gaitState[j] = servoRangeOfMotion
        pos.extend([duration] + gaitState)
    print(len(pos))
    return [duration] + [force] + pos


def readGait(progress, chromosome):
    global firstCycleComplete
    end_index = LENGTH_OF_SEQUENCE
    if not firstCycleComplete and progress >= sum([chromosome[x] for x in range(0, len(chromosome), LENGTH_OF_GAIT_STATE)]):
        firstCycleComplete = True
    if firstCycleComplete:
        progress = progress - sum([chromosome[x] for x in range(0, ((LENGTH_OF_START_SEQUENCE - 1) * LENGTH_OF_GAIT_STATE) + 1, LENGTH_OF_GAIT_STATE)])
        chromosome = chromosome[LENGTH_OF_START_SEQUENCE * LENGTH_OF_GAIT_STATE:]
        end_index = LENGTH_OF_CYCLE
    start_index = 0
    total_duration = sum([chromosome[x] for x in range(0, len(chromosome), LENGTH_OF_GAIT_STATE)])
    # duration_of_start_sequence = sum([chromosome[x] for x in range(0, ((LENGTH_OF_START_SEQUENCE - 1) * LENGTH_OF_GAIT_STATE) + 1, LENGTH_OF_GAIT_STATE)])
    # duration_of_cycle = total_duration - duration_of_start_sequence
    progress = progress % total_duration
    current_duration_index = 0
    next_duration_index = 0
    sum_of_durations = 0
    for j in range(start_index, end_index):
        current_position_index = j * LENGTH_OF_GAIT_STATE
        sum_of_durations = sum([chromosome[x] for x in range(start_index, current_position_index + 1, LENGTH_OF_GAIT_STATE)])
        if progress < sum_of_durations:
            current_duration_index = current_position_index
            next_duration_index = (j + 1) * LENGTH_OF_GAIT_STATE
            if (j + 1) >= end_index:
                next_duration_index = start_index * LENGTH_OF_GAIT_STATE
            break
    current_gait_state = chromosome[current_duration_index + 1: current_duration_index + LENGTH_OF_GAIT_STATE]
    next_gait_state = chromosome[next_duration_index + 1: next_duration_index + LENGTH_OF_GAIT_STATE]
    if not firstCycleComplete and current_duration_index == (LENGTH_OF_SEQUENCE - 1) * LENGTH_OF_GAIT_STATE:
        next_gait_state = chromosome[(LENGTH_OF_START_SEQUENCE * LENGTH_OF_GAIT_STATE) + 1: (LENGTH_OF_START_SEQUENCE * LENGTH_OF_GAIT_STATE) + LENGTH_OF_GAIT_STATE]
    alpha = (progress - (sum_of_durations - chromosome[current_duration_index])) / chromosome[current_duration_index]
    interpolated_gait_state = [interpolate(a, b, alpha) for a, b in zip(current_gait_state, next_gait_state)]
    servoPositions = convertTripodChromosomeToFull(interpolated_gait_state)
    return servoPositions


def convertTripodChromosomeToFull(ind):
    frontRightLeg = ind[:3]
    backRightLeg = frontRightLeg
    middleLeftLeg = [-x for x in frontRightLeg]
    middleRightLeg = ind[3:]
    frontLeftLeg = [-x for x in middleRightLeg]
    backLeftLeg = frontLeftLeg
    fullServoChromosome = frontRightLeg + middleRightLeg + backRightLeg + frontLeftLeg + middleLeftLeg + backLeftLeg
    return fullServoChromosome


def interpolate(a, b, alpha):
    return a * (1 - alpha) + b * alpha


def resetLegJoints():
    p.resetJointStatesMultiDof(hexapod_ID, JOINT_INDEXES, [[0]] * 18, targetVelocities=[[0]] * 18)
    p.setJointMotorControlArray(hexapod_ID, JOINT_INDEXES, p.POSITION_CONTROL, targetPositions=([0] * 18), forces=([150] * 18))


def resetEnvironment():
    resetLegJoints()
    p.resetBasePositionAndOrientation(hexapod_ID, [0, STARTING_Y, STARTING_HEIGHT + random.uniform(0, 0.002)], [0, 0, 0, 1])
    p.stepSimulation()


def resetPyBulletSimulation():
    global plane_ID
    global hexapod_ID
    p.resetSimulation()
    p.setGravity(0, 0, -9.8)
    plane_ID = p.loadURDF("plane.urdf", globalScaling=4)
    # testAngle = p.getQuaternionFromEuler([0, math.pi/2, math.pi])
    hexapod_ID = p.loadURDF("robot3.urdf", [0, STARTING_Y, STARTING_HEIGHT + random.uniform(0, 0.002)], [0, 0, 0, 1])
    print(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(hexapod_ID)[1]))


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


def inverseCurve(x, a):
    # 1/e^2 = 0.135
    # a = 0.135
    # a = 1
    # (pi/2.0)^2 = 2.467
    # a = 2.467
    y = a / (a + (x * x))
    return y


def collidingLegs():
    # numOfCollisions = 0
    for j in range(24):
        aabb = (p.getAABB(hexapod_ID, j))
        familyOfLinks = [x for x in range(24) if math.floor(j / 4) == math.floor(x / 4)] + [-1]
        collisionObjects = [x[1] for x in p.getOverlappingObjects(aabb[0], aabb[1]) if (j not in FEET_INDEXES and (x[1] not in familyOfLinks or x[0] != hexapod_ID)) or (j in FEET_INDEXES and x[1] not in familyOfLinks and x[0] == hexapod_ID)]
        if len(collisionObjects) > 0:
            return True
    return False


def runGait(individual):
    # ssc32 = serial.Serial('COM5', 115200, timeout=2)  # open serial port
    lastTime = time.time()
    global firstCycleComplete
    dt = 0
    firstCycleComplete = False
    initDuration = individual[0]
    force = individual[1]
    gaitChromosome = individual[2:]
    gaitChromosome = ([initDuration] + [0] * NUM_OF_SERVOS) + gaitChromosome
    resetEnvironment()
    p.setRealTimeSimulation(1)
    while True:
        p.setJointMotorControlArray(hexapod_ID, JOINT_INDEXES, p.POSITION_CONTROL, targetPositions=readGait(dt, gaitChromosome), forces=([force] * 18))
        # p.setJointMotorControlArray(hexapod_ID, JOINT_INDEXES, p.POSITION_CONTROL, targetPositions=read_debug_parameters(), forces=([force] * 18))
        # updateRealServos(ssc32, 100)
        now = time.time()
        dt += now - lastTime
        lastTime = now
        # print(collidingLegs())


def evaluateGait(individual):
    lastTime = time.time()
    numOfPhysicsSteps = 3000
    samplesPerEval = 100
    stabilityUpdateRate = int(numOfPhysicsSteps / samplesPerEval)
    stabilityScore = 0
    heightScore = 0
    collisionScore = 0
    global firstCycleComplete
    while True:
        dt = 0
        firstCycleComplete = False
        initDuration = individual[0]
        force = individual[1]
        gaitChromosome = individual[2:]
        gaitChromosome = ([initDuration] + [0] * NUM_OF_SERVOS) + gaitChromosome
        resetEnvironment()
        for ii in range(numOfPhysicsSteps):
            if ii % stabilityUpdateRate == 0:
                hexapodBasePosAndOrn = p.getBasePositionAndOrientation(hexapod_ID)
                currentStability = sum([abs(angle) for angle in list(p.getEulerFromQuaternion(hexapodBasePosAndOrn[1]))])
                currentHeight = abs(TARGET_HEIGHT - hexapodBasePosAndOrn[0][2])
                stabilityScore += currentStability
                heightScore += currentHeight
                collisionScore += collidingLegs()
            p.setJointMotorControlArray(hexapod_ID, JOINT_INDEXES, p.POSITION_CONTROL, targetPositions=readGait(dt, gaitChromosome), forces=([force] * 18))
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


# start of main program
MAX_MOTIONS_IN_SEQUENCE = 4
NUM_OF_LEGS = 6
NUM_OF_JOINTS_PER_LEG = 3
# NUM_OF_SERVOS = NUM_OF_LEGS * NUM_OF_JOINTS_PER_LEG
NUM_OF_SERVOS = NUM_OF_JOINTS_PER_LEG * 2
UNIQUE_THREAD_ID = random.randint(1, 10000)
LENGTH_OF_CYCLE = 12
LENGTH_OF_START_SEQUENCE = 2 + 1
LENGTH_OF_SEQUENCE = LENGTH_OF_START_SEQUENCE + LENGTH_OF_CYCLE
LENGTH_OF_GAIT_STATE = NUM_OF_SERVOS + 1
STARTING_HEIGHT = 1.375
STARTING_Y = 0.01
TARGET_HEIGHT = STARTING_HEIGHT
firstCycleComplete = False
control_IDs = []

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
JOINT_INDEXES = [x for x in range(0, 24) if (x + 1) % 4 != 0]
FEET_INDEXES = [x for x in range(0, 24) if (x + 1) % 4 == 0]
p.setRealTimeSimulation(0)

print(f'PyBullet Instance ID: {UNIQUE_THREAD_ID}')


def main():
    init_debug_parameters()
    print("START")
    tc = [1] + [150] + ([1] + ([0] * 6)) * (LENGTH_OF_SEQUENCE - 1)
    testChromosome = chromosomeCreator()
    runGait([0.328729446347865, 156.50518886212103, 0.20521635627231335, -0.14553306293498203, -1.0736929579294912, -0.034905351955989844, -0.34571798265419923, 0.04668546761510084, -0.2499295089305258, 0.289729352943405, 0.23904057563980546, -0.08177149561338878, 0.49474219468453584, -0.15531089629114855, -0.19681586456200156, 1.1423386812599203, 0.20266337438887996, -0.347202591616732, 0.10335286922435301, -0.3095802492724863, 0.09015762014547135, -0.6368480293859653, 0.3839029782961861, 0.20136116735843215, -0.19978464674395285, -0.3006271714749286, 0.46858368483271723, 0.1430470730099945, -0.013830592569564206, 0.043058222328504606, 0.20078212021498676, 0.1366369323688486, -0.03793790478835023, -0.4033928595773487, -0.31674619266943377, 0.03377845138957706, -0.33079201402961683, 0.2609878828787827, 0.1092155089197386, -0.02886512763822693, 0.1407003188894461, -0.3046888622330375, -0.034909904929206044, 0.1471038991641587, 0.20502534210853224, -0.3466604982930702, 0.08358578016578032, -0.3751968619252107, 0.19848102156218903, -0.15103031191310218, 0.15239784943566953, 0.20912079409980827, -0.025903298419364985, -1.1337270128856154, 0.26187230706286396, -0.4501462566699238, 0.12972598194132798, -0.46551422314883073, 0.20212447022100086, 0.1296942846345587, -0.007789696296051595, -0.32444196926471663, -0.18081280569243238, -0.20420070143668756, 0.43464264344666176, 0.20259890997061902, -0.3218114812325167, 0.13361051616434624, -0.21765016248077573, 0.20024258397888386, -0.14137007303129157, 0.22619838777039503, 0.2037333663525923, -0.035847935122647924, -1.1189656705522162, -0.7957916601166352, -0.46010061416750764, 0.0430010286689714, -0.38482680559972404, 0.20314846073891954, 0.05801106550576211, 0.20688834355781216, -0.24501381660342192, 0.07548165493347389, -1.0215135632638601, 0.1565496404994422, 0.20387886412825987, -0.34460990068964187, -0.11647007940084053, 0.23056977084281233, 0.08794559405970703, 0.013499174544767007, -0.15663712185485384, 0.2019116676826425, 0.21999621144228126, -0.05602697382591249, 0.23436182733434943, -0.37772272589080297, 0.045745077684371915, -0.2356667141035915])
    p.disconnect(physicsClient)


if __name__ == "__main__":
    main()
