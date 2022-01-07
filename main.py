import pybullet as p
import math
import numpy as np
import pybullet_data
import time
import bezier


def init_debug_parameters():
    for j in list(range(0, 6)):
        control_IDs.append(p.addUserDebugParameter(f"Pelvis {j}", -servoRangeOfMotion / 2, servoRangeOfMotion / 2, 0))
        control_IDs.append(p.addUserDebugParameter(f"Hip {j}", -servoRangeOfMotion / 2, servoRangeOfMotion / 2, 0))
        control_IDs.append(p.addUserDebugParameter(f"Knee {j}", -servoRangeOfMotion / 2, servoRangeOfMotion / 2, 0))


def read_debug_parameters():
    angles = []
    for x in control_IDs:
        angles.append(p.readUserDebugParameter(x))
    return angles


def calculateIK2():
    debug_parameters = np.array([
        p.readUserDebugParameter(xPara),
        p.readUserDebugParameter(yPara),
        p.readUserDebugParameter(zPara)])

    pos_body = np.array(p.getBasePositionAndOrientation(hexapod_ID)[0])
    orn_body = p.getBasePositionAndOrientation(hexapod_ID)[1]

    speed = 3
    rest_poses = []
    local_target_pos = []
    translated_pos = []
    target_pos = []

    # bezier curve
    # nodes1 = np.array([[1.0, 1.5, 0.5, 0.0], [0.0, 0.5, 1.0, 0.0]])
    nodes1 = np.array([[0.8, 1.2, 0.0, 0.0], [-0.5, 0.5, 1.5, 0.5], [0.0, 1.0, 0.5, 0.0]])
    curve1 = bezier.Curve(nodes1, degree=3)

    for j in range(6):
        rest_poses.append(
            pos_body + p.multiplyTransforms([0, 0, 0], orn_body, baseToEndEffectorConstVec[j], [0, 0, 0, 1])[0])
        # local_target_pos.append((np.array([math.cos(-dt * speed), math.sin(-dt * speed), 0]) * 0.4) + debug_parameters)
        if j == 0 or j == 2 or j == 4:
            curve_speed = 0.25
            curve_size = 0.5
            bezier_curve_pos = np.array(curve1.evaluate((dt * curve_speed) % 1)) * curve_size
            print(bezier_curve_pos)
            local_target_pos.append((np.array([bezier_curve_pos[0][0], bezier_curve_pos[1][0], bezier_curve_pos[2][0]])))
        else:
            local_target_pos.append(debug_parameters)
            # local_target_pos.append([0,0,-0.5])

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


def debugDisplay():
    return 0


def setServoStatesManual():
    p.setJointMotorControlArray(
        hexapod_ID,
        ([x for x in list(range(0, 24)) if x not in list(range(3, 24, 4))]),
        p.POSITION_CONTROL,
        targetPositions=read_debug_parameters())


def setServoStatesLegs():
    p.setJointMotorControlArray(
        hexapod_ID,
        ([x for x in list(range(0, 24)) if x not in list(range(3, 24, 4))]),
        p.POSITION_CONTROL,
        targetPositions=calculateIK2(),
        forces=([150] * 18)
    )


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
hexapod_ID = p.loadURDF("robot.urdf", [0, 0, 1.4], p.getQuaternionFromEuler([0, 0, 0]))

control_IDs = []
servoRangeOfMotion = math.pi * 3 / 4
# init_debug_parameters()

# setup IK parameters
ll = ([-servoRangeOfMotion] * 3) + ([0] * 15)  # lowerLimit
ul = ([servoRangeOfMotion] * 3) + ([0] * 15)  # upperLimit
jr = ([servoRangeOfMotion] * 3) + ([0] * 15)  # jointRange
rp = ([0] * 18)  # restPos
jd = ([5] * 18)  # jointDamping (like a PID controller)

baseToEndEffectorConstVec = []
for i in range(3, 24, 4):
    baseToEndEffectorConstVec.append(
        np.array(p.getLinkState(hexapod_ID, i)[4]) - np.array(p.getBasePositionAndOrientation(hexapod_ID)[0]))

testLegID = 3

xPara = p.addUserDebugParameter("X", -3, 3, 0)
yPara = p.addUserDebugParameter("Y", -3, 3, 0)
zPara = p.addUserDebugParameter("Z", -3, 3, 0)
rotationPara = p.addUserDebugParameter("Rotation", -math.pi, math.pi, 0)

prevPos = []
currentPos = [0, 0, 0, 0, 0, 0]
for i in range(6):
    prevPos.append(p.getLinkState(hexapod_ID, (4 * i) + 3)[0])

lastTime = time.time()

while True:
    dt = time.time() - lastTime
    setServoStatesLegs()

    for i in range(6):
        currentPos[i] = p.getLinkState(hexapod_ID, (4 * i) + 3)[0]
        p.addUserDebugLine(prevPos[i], currentPos[i], [0, 0, 0.3], 4, 5)
        prevPos[i] = currentPos[i]
    p.setRealTimeSimulation(1)
    # p.stepSimulation()
    # time.sleep(1. / 240.)

p.disconnect(physicsClient)
