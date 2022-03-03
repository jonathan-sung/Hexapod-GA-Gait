
JOINT_INDICES = [x for x in range(0, 24) if (x + 1) % 4 != 0]
FEET_INDEXES = [x for x in range(0, 24) if (x + 1) % 4 == 0]
print(JOINT_INDICES)
print(FEET_INDEXES)
