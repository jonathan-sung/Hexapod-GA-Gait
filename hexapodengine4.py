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
        f'#0P{radToPwm(-p.getJointState(hexapod_ID, 8)[0])}T{t}#1P{radToPwm(p.getJointState(hexapod_ID, 9)[0])}T{t}#2P{radToPwm(-p.getJointState(hexapod_ID, 10)[0])-100}T{t}\r'.encode(
            'utf-8'))
    ser.write(
        f'#4P{radToPwm(-p.getJointState(hexapod_ID, 4)[0])}T{t}#5P{radToPwm(p.getJointState(hexapod_ID, 5)[0])}T{t}#6P{radToPwm(-p.getJointState(hexapod_ID, 6)[0])+100}T{t}\r'.encode(
            'utf-8'))
    ser.write(
        f'#8P{radToPwm(-p.getJointState(hexapod_ID, 0)[0])}T{t}#9P{radToPwm(p.getJointState(hexapod_ID, 1)[0])}T{t}#10P{radToPwm(-p.getJointState(hexapod_ID, 2)[0])}T{t}\r'.encode(
            'utf-8'))

    # left legs
    ser.write(
        f'#24P{radToPwm(-p.getJointState(hexapod_ID, 12)[0])}T{t}#25P{radToPwm(p.getJointState(hexapod_ID, 13)[0])}T{t}#26P{radToPwm(-p.getJointState(hexapod_ID, 14)[0])+100}T{t}\r'.encode(
            'utf-8'))
    ser.write(
        f'#20P{radToPwm(-p.getJointState(hexapod_ID, 16)[0])}T{t}#21P{radToPwm(p.getJointState(hexapod_ID, 17)[0])}T{t}#22P{radToPwm(-p.getJointState(hexapod_ID, 18)[0])}T{t}\r'.encode(
            'utf-8'))
    ser.write(
        f'#16P{radToPwm(-p.getJointState(hexapod_ID, 20)[0])}T{t}#17P{radToPwm(p.getJointState(hexapod_ID, 21)[0])}T{t}#18P{radToPwm(-p.getJointState(hexapod_ID, 22)[0])-50}T{t}\r'.encode(
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
    hexapod_ID = p.loadURDF("robot2.urdf", [0, STARTING_Y, STARTING_HEIGHT + random.uniform(0, 0.002)], [0, 0, 0, 1])
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
    global REAL_HEXAPOD_CONNECTED
    lastTime = time.time()
    global firstCycleComplete
    dt = 0
    firstCycleComplete = False
    initDuration = individual[0]
    force = individual[1]
    gaitChromosome = individual[2:]
    gaitChromosome = ([initDuration] + [0] * NUM_OF_SERVOS) + gaitChromosome
    resetEnvironment()
    stabilityScore = 0
    heightScore = 0
    collisionScore = 0
    sampleCounter = 0
    p.setRealTimeSimulation(1)
    while True:
        if CONFIG_MODE:
            p.setJointMotorControlArray(hexapod_ID, JOINT_INDEXES, p.POSITION_CONTROL, targetPositions=read_debug_parameters(), forces=([force] * 18))
        else:
            p.setJointMotorControlArray(hexapod_ID, JOINT_INDEXES, p.POSITION_CONTROL, targetPositions=readGait(dt, gaitChromosome), forces=([force] * 18))
        if REAL_HEXAPOD_CONNECTED:
            updateRealServos(ssc32, 100)

        # Evaluation Metrics
        hexapodBasePosAndOrn = p.getBasePositionAndOrientation(hexapod_ID)
        currentStability = sum([abs(angle) for angle in list(p.getEulerFromQuaternion(hexapodBasePosAndOrn[1]))])
        currentHeight = abs(1.375 - hexapodBasePosAndOrn[0][2])
        stabilityScore += currentStability
        heightScore += currentHeight
        #collisionScore += collidingLegs()
        sampleCounter += 1

        # timing variables
        now = time.time()
        dt += now - lastTime
        lastTime = now

        # Finish evaluation after 12.5 seconds
        if dt >= 12.5 and not INFINTE_RUN:
            break

    hexapodBasePosAndOrn = p.getBasePositionAndOrientation(hexapod_ID)
    currentPosition = hexapodBasePosAndOrn[0]
    distance = hexapodBasePosAndOrn[0][1]
    straightness = abs(angleBetweenVectors(np.array([0, 1]), np.array([currentPosition[0], currentPosition[1]])))
    avgHeight = abs(heightScore / sampleCounter)
    avgStability = stabilityScore / sampleCounter
    avgNumOfCollisions = collisionScore / sampleCounter
    fitness_distance = distance / 100.0
    fitness_straight = 1.0 - (straightness / math.pi)
    fitness_stability = inverseCurve(avgStability, 1)
    fitness_height = inverseCurve(avgHeight, 1)
    fitness_collisions = round(1 - avgNumOfCollisions, 2)
    fitness_total = (fitness_distance + fitness_straight + fitness_stability + fitness_height + fitness_collisions) / 5.0
    line = f'ID: {UNIQUE_THREAD_ID} | Time Elapsed: {dt} | Evaluation: {fitness_distance, fitness_straight, fitness_stability, fitness_height, fitness_collisions, fitness_total} | Chromosome: {individual}'
    print(line)
    with open('C:/Users/Jonathan/Desktop/results_tripod_cyclic.txt', 'a') as f:
        f.write(line)
        f.write('\n')
    return fitness_total


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
        line = f'ID: {UNIQUE_THREAD_ID} | Time Elapsed: {time.time() - lastTime} | Evaluation: {fitness_distance, fitness_straight, fitness_stability, fitness_height, fitness_collisions, fitness_total} | Chromosome: {individual}'
        print(line)
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
NUM_OF_SERVOS = NUM_OF_JOINTS_PER_LEG * 2  # tripod variation
UNIQUE_THREAD_ID = random.randint(1, 10000)
#LENGTH_OF_CYCLE = 12
LENGTH_OF_CYCLE = 5
LENGTH_OF_START_SEQUENCE = 2 + 1
LENGTH_OF_SEQUENCE = LENGTH_OF_START_SEQUENCE + LENGTH_OF_CYCLE
LENGTH_OF_GAIT_STATE = NUM_OF_SERVOS + 1
STARTING_HEIGHT = 1.375
STARTING_Y = 0.01
TARGET_HEIGHT = STARTING_HEIGHT
firstCycleComplete = False
control_IDs = []
REAL_HEXAPOD_CONNECTED = False
CONFIG_MODE = False
INFINTE_RUN = False
ssc32 = None
if REAL_HEXAPOD_CONNECTED:
    ssc32 = serial.Serial('COM3', 115200, timeout=2)  # open serial port

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
    # runGait([0.358261765350468, 200, 0.21114768095792824, 0.03689819593079225, -0.9004553418992695, -0.0027600167978495405, -0.35766807754667596, 0.026228075419628426, -0.27911911790849303, 0.31181404973792315, 0.2551819380367277, -0.07988079079472789, 0.4762471443822096, -0.1742794371974889, -0.10880481062410981, 0.8070106640040263, 0.20015717642912206, -0.3415335227192982, 0.10407187327081428, -0.34155787594826553, 0.09069889462763335, -0.6621239256577818, 0.29053703338088543, 0.20095244595624365, -0.18955911553657148, -0.32598366967870834, 0.16273478558166043, 0.18200495065999256, -0.03543169746486387, 0.08009517396365508, 0.2001210481494467, 0.1553466410510423, -0.0480703061898969, -0.38153164835542125, -0.3185723672940789, 0.06011509078579546, -0.32393342601686426, 0.26331708434583573, 0.1038715352618737, -0.03188498519478791, 0.14971273726168008, -0.3292861115253225, -0.036093368931868895, 0.06521404973200823, 0.20118127834045801, -0.34925518702243413, 0.06978902110612184, -0.36527051067796373, 0.20057493243643856, -0.14198292555662556, 0.19113299130447722, 0.21027225839322417, -0.0417565612727313, -1.070884549836226, 0.0544280474656791, -0.4801830777298174, 0.12082959298919985, -0.4577381111435565, 0.2039427894951525, 0.1978490857000423, 0.024365423707423336, -0.31547012931811286, -0.19426109131041708, -0.2255525837617507, 0.3932779653997114, 0.20082553792917074, -0.36290637413137955, 0.1136824032128516, -0.20714803319458316, 0.21174865611730243, -0.13647606252351938, 0.22257091941426668, 0.20353760883137725, -0.03011767598379143, -1.1501202229837035, -0.875585762810224, -0.5182540408997005, 0.04287027664660465, -0.389710758181074, 0.2047307929678799, 0.0492157529393039, 0.2509677193326165, -0.351409038231271, 0.046772514207063035, -1.0812599234649334, 0.24282179542679075, 0.20242740533921963, -0.3686492585494665, -0.12632636046701018, 0.405108332357141, 0.07865245406602453, 0.028041650554219512, -0.18693978492502086, 0.2020905818424943, 0.23600669594896814, -0.05313631102541742, 0.25326022106656826, -0.3890286814386256, 0.05334226401869782, -0.1912063041670727])
    # runGait([0.21502962703794637, 127.61045339572054, 0.26584273191412555, 0.49357451902526145, -0.33558797648776545, 0.4828677334449464, -0.27928934100470476, 0.3737524632726193, -0.13587581133170729, 0.20653030910488898, 0.33366350026581637, 0.5795204783955112, -0.25432849448627926, -0.05192028257942284, -0.050076056810303776, 0.08219735422888255, 0.38768875762791977, 0.17512283017413546, -0.4667579775275463, 0.6168085806757475, -0.48922701538800173, -0.37998534120532623, 0.5237981018876466, 0.2066277728655636, -0.037816649529037516, -0.0018649974804514635, 0.26332445339994104, 0.37156618836483424, -0.3769705772402279, 0.1536766019034156, 0.20949442505305949, -0.4394231068584433, -0.2737970030753856, 0.22876366194613482, 0.6606472085755586, -0.4654811135548855, 0.49320762620598363, 0.213168144402951, -0.6346333986801331, -0.14677023706206885, 0.17482898580924228, 0.22703624650738546, 0.12869599848598232, 0.08009120739035605, 0.20468152345974389, 0.3746042412667161, -1.0753887650833112, 0.46136900564889655, 0.09780526346503478, -0.22110494581281911, 0.31058769932965735, 0.2732110634408548, 0.46328301812163364, 0.014668841597035357, 0.06669945848670612, -0.705089724555003, 0.14412150957599432, -0.14583057549083106, 0.23462788299098494, -0.12308936124610928, 0.13363241570345216, -0.33273710537709583, 0.4078781202172792, -0.848123636643898, 0.7014989000481779, 0.23209196426397635, -0.7203664752070424, -0.18012354946660789, 0.1828500169231529, 0.29835332718829055, 0.13571368072622236, 0.055278449970895635, 0.21747991991417007, 0.40306415528018613, -0.33273735620048855, 0.30544103472946826, -0.44654361042654034, 0.09229196684698221, -0.6097372608840812, 0.20192838934406437, 0.16247911050551458, 0.23923959794662064, -0.18985760460960066, -0.09903558710410572, -0.26805116688023317, 0.08482083940124877, 0.2984959209558379, -0.4352038702223257, 0.12411895025253439, -0.45173790151253745, 0.4459606850435182, -0.09662747873801057, 0.11763333983803016, 0.2747094261580951, 0.43036371465961554, -0.2606583028736393, -0.061019052714426286, -0.41953946206130305, 0.3745598867363541, -0.6178220953642066])
    runGait([0.22202767193511108, 129.96065626165588, 0.4106932581683045, -0.44064703296335045, 0.06424853485338923, -0.4203120988670678, 0.4850225728472963, -0.356086953196741, 0.03034343750209318, 0.24816421770667116, -0.16102151497477707, -0.00797554827986247, 0.27880690281104037, 0.10580757810278475, 0.8229241274243086, -0.19919882462053726, 0.2003475425835074, -0.18487964201829118, 0.15489242048729598, -0.45852673718075265, 0.08766105894978077, -0.038249496253938836, -0.14761046158150123, 0.2007592012321721, 0.6904334591414614, -0.14403474485286027, 0.18676006615828342, -0.24707765588331845, 0.18217228756989404, -0.15826358681188496, 0.3080870706683921, 0.39094382113564935, -0.021212162581330064, 0.043931496298091045, -0.6247245127433452, -0.042774691700302916, 0.09357604327868055, 0.3342352277160678, -0.5527202957981088, 0.07726123982750599, -0.545133275714376, 0.6117411828853526, -0.1436158941507929, 0.12951688564131053, 0.20426060013960903, 0.3009488354785699, -0.32427259568390954, 0.9768465293622431, -0.3515596986667522, 0.24889893331608698, -0.7908348979107827])
    p.disconnect(physicsClient)


if __name__ == "__main__":
    main()
