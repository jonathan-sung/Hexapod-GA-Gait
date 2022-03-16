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
    p.setGravity(0, 0, -20)
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
            # print("collision", time.time())
            return True
    return False


def runGait(individual):
    #ssc32 = serial.Serial('COM5', 115200, timeout=2)  # open serial port
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
        #p.setJointMotorControlArray(hexapod_ID, JOINT_INDEXES, p.POSITION_CONTROL, targetPositions=readGait(dt, gaitChromosome), forces=([force] * 18))
        p.setJointMotorControlArray(hexapod_ID, JOINT_INDEXES, p.POSITION_CONTROL, targetPositions=read_debug_parameters(), forces=([force] * 18))
        #updateRealServos(ssc32, 100)
        # collidingLegs()
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
    #runGait([0.358261765350468, 200, 0.21114768095792824, 0.03689819593079225, -0.9004553418992695, -0.0027600167978495405, -0.35766807754667596, 0.026228075419628426, -0.27911911790849303, 0.31181404973792315, 0.2551819380367277, -0.07988079079472789, 0.4762471443822096, -0.1742794371974889, -0.10880481062410981, 0.8070106640040263, 0.20015717642912206, -0.3415335227192982, 0.10407187327081428, -0.34155787594826553, 0.09069889462763335, -0.6621239256577818, 0.29053703338088543, 0.20095244595624365, -0.18955911553657148, -0.32598366967870834, 0.16273478558166043, 0.18200495065999256, -0.03543169746486387, 0.08009517396365508, 0.2001210481494467, 0.1553466410510423, -0.0480703061898969, -0.38153164835542125, -0.3185723672940789, 0.06011509078579546, -0.32393342601686426, 0.26331708434583573, 0.1038715352618737, -0.03188498519478791, 0.14971273726168008, -0.3292861115253225, -0.036093368931868895, 0.06521404973200823, 0.20118127834045801, -0.34925518702243413, 0.06978902110612184, -0.36527051067796373, 0.20057493243643856, -0.14198292555662556, 0.19113299130447722, 0.21027225839322417, -0.0417565612727313, -1.070884549836226, 0.0544280474656791, -0.4801830777298174, 0.12082959298919985, -0.4577381111435565, 0.2039427894951525, 0.1978490857000423, 0.024365423707423336, -0.31547012931811286, -0.19426109131041708, -0.2255525837617507, 0.3932779653997114, 0.20082553792917074, -0.36290637413137955, 0.1136824032128516, -0.20714803319458316, 0.21174865611730243, -0.13647606252351938, 0.22257091941426668, 0.20353760883137725, -0.03011767598379143, -1.1501202229837035, -0.875585762810224, -0.5182540408997005, 0.04287027664660465, -0.389710758181074, 0.2047307929678799, 0.0492157529393039, 0.2509677193326165, -0.351409038231271, 0.046772514207063035, -1.0812599234649334, 0.24282179542679075, 0.20242740533921963, -0.3686492585494665, -0.12632636046701018, 0.405108332357141, 0.07865245406602453, 0.028041650554219512, -0.18693978492502086, 0.2020905818424943, 0.23600669594896814, -0.05313631102541742, 0.25326022106656826, -0.3890286814386256, 0.05334226401869782, -0.1912063041670727])
    runGait([0.5098692951144865, 124.02194428582598, 0.8040454032230343, 0.35739897999634684, 0.1722596115347097, 0.5101257035303509, -0.39633084864971463, 0.3766339631565424, 0.6852944512751009, 0.8579549777676551, -0.15192012790045337, 0.12057372078796795, -0.639336374853795, -0.4989247924699875, -0.05520693989007108, 0.7616287343423316, 0.7970307001779818, -0.34788633377110717, -0.016427459332195315, 0.549842994935035, 0.4777775877037007, -0.4701569379170369, 0.6308243915293652, 0.7120163151872847, 0.1812869824351379, -0.141241427911283, 0.04151767772880015, -0.38802211681528925, 0.4047659386635642, -0.08658252502441663, 0.33328294677530673, -0.13477632123802907, 0.2649278124913589, 0.013747744433013272, 0.7198655323331445, -0.36845153212178955, 0.5502953359095913, 0.48527438391600675, -0.192846496531068, -0.40021506534462664, 0.30351238554707705, 0.49711549344836437, -0.04280555536924904, 0.41124814747484323, 0.588412525102943, 0.20683907668827006, -0.18237704309959019, 0.20652066894382617, -0.5422015779265399, -0.07745611292226864, 0.005939299135961668, 0.38333363384956337, 0.13383678739049448, -0.09501187735535857, 0.5592676334964294, -0.7250321063232774, -0.21756905859589573, 0.3535429124591077, 0.7193081079858578, -0.3490361246167593, -0.2899820874997728, 0.12978350241500172, 0.5786060086448825, -0.8215247817139338, 0.6962575326576299, 0.8170135955134692, -0.5420910512422592, 0.16722490131071788, 0.22849241933682138, -0.6260322651673402, -0.22911216712744673, 0.9948562233197314, 0.28702329648755515, -0.12380025800216428, 0.04056270230350641, 1.0698182320147702, -0.05192953965904736, -0.4657016874860602, 1.127764516248737, 0.8388768987394088, 0.06433659503168734, -1.0568145519578818, 0.9078672501839635, -0.3970462108379715, -0.33578497211959146, -0.12551083915385197, 0.20419534756280153, 0.13303090611111554, 1.1034391693101104, -0.8174812898759956, -0.30173151556638056, 0.8581279830178448, 0.20629269748256712, 0.22083383509788582, 0.06314771218306436, 0.9254903847014039, -0.1060367095740995, -0.16790883918756827, -0.7389988355839133, 0.12541546845817492])
    p.disconnect(physicsClient)


if __name__ == "__main__":
    main()
