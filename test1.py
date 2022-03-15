import math


def sinusoidalTestGait(t):
    coxa0 = (math.pi / 4) * math.sin((2 * t) + math.pi)
    femur0 = 0.2 * math.sin((2 * t) + ((5 * math.pi) / 2))
    tibia0 = 1.3 * math.sin((0 * t) + ((3 * math.pi) / 2))
    coxa1 = (math.pi / 4) * math.sin((2 * t) + 0)
    femur1 = 0.2 * math.sin((2 * t) + ((3 * math.pi) / 2))
    tibia1 = 1.3 * math.sin((0 * t) + ((3 * math.pi) / 2))
    return [coxa0, femur0, tibia0, coxa1, femur1, tibia1] * 3
