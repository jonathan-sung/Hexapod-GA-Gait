import pybullet as p
import math
import pybullet_data
import time
import random


def chromosomeCreator():
    pos = []
    duration = 0.5
    # pos.extend([duration] + [0] * NUM_OF_SERVOS)
    for j in range(LENGTH_OF_SEQUENCE):
        gaitState = [0] * NUM_OF_SERVOS
        gaitState[j] = servoRangeOfMotion
        pos.extend([duration] + gaitState)
    print(len(pos))
    return pos


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
    # print(progress, alpha, sum_of_durations, chromosome[current_duration_index])
    return interpolated_gait_state


def interpolate(a, b, alpha):
    return a * (1 - alpha) + b * alpha


def resetLegJoints():
    p.resetJointStatesMultiDof(hexapod_ID, JOINT_INDICES, [[0]] * 18, targetVelocities=[[0]] * 18)
    p.setJointMotorControlArray(hexapod_ID, JOINT_INDICES, p.POSITION_CONTROL, targetPositions=([0] * 18), forces=([150] * 18))


def resetEnvironment():
    resetLegJoints()
    p.resetBasePositionAndOrientation(hexapod_ID, [0, 0, 1.375 + random.uniform(0, 0.002)], [0, 0, 0, 1])
    p.stepSimulation()


def resetPyBulletSimulation():
    global plane_ID
    global hexapod_ID
    p.resetSimulation()
    p.setGravity(0, 0, -9.8)
    plane_ID = p.loadURDF("plane.urdf", globalScaling=4)
    hexapod_ID = p.loadURDF("robot2.urdf", [0, 0, 1.375 + random.uniform(0, 0.002)], [0, 0, 0, 1])


def runGait(individual):
    p.setRealTimeSimulation(1)
    dt = 0
    lastTime = time.time()
    global firstCycleComplete
    firstCycleComplete = False
    while True:
        p.setJointMotorControlArray(hexapod_ID, JOINT_INDICES, p.POSITION_CONTROL, targetPositions=readGait(dt, individual), forces=([300] * 18))
        now = time.time()
        dt += now - lastTime
        lastTime = now


# start of main program
MAX_MOTIONS_IN_SEQUENCE = 4
NUM_OF_LEGS = 6
NUM_OF_JOINTS_PER_LEG = 3
NUM_OF_SERVOS = NUM_OF_LEGS * NUM_OF_JOINTS_PER_LEG
UNIQUE_THREAD_ID = random.randint(1, 10000)
LENGTH_OF_CYCLE = 12
LENGTH_OF_START_SEQUENCE = 3
LENGTH_OF_SEQUENCE = LENGTH_OF_START_SEQUENCE + LENGTH_OF_CYCLE
LENGTH_OF_GAIT_STATE = NUM_OF_SERVOS + 1
firstCycleComplete = False

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
JOINT_INDICES = [x for x in range(0, 24) if (x + 1) % 4 != 0]

print(f'PyBullet Instance ID: {UNIQUE_THREAD_ID}')


def main():
    print("START")
    resetEnvironment()
    runGait(chromosomeCreator())
    p.disconnect(physicsClient)


if __name__ == "__main__":
    main()
