import math
import numpy as np
import bezier
from matplotlib import pyplot as plt


# nodes1 = np.asfortranarray([[0.0, 0.0, 1.0], [0.0, 1.0, 1.0], ])
# curve1 = bezier.Curve(nodes1, degree=2)
# curve1.plot(num_pts=256)
# print(curve1.evaluate(0.5))

def multi(ls, f):
    return [ls[0] * f, ls[1] * f]


# SIMULATIONS - *1000
# foo0 = 0.153703311,	0.016815957
# foo1 = 0.249335272, 0.049826293
# foo2 = 0.247732819,	0.062630561

# REALITY
# foo0 = 137.3133011,	6.615459575
# foo1 = 117.770247,	10.91504581
# foo2 = 133.8555261,	-8.282580371

# Manual
foo0 = 0.074818725, 0.021156662
foo1 = -0.011818946, 0.014310335
foo2 = 0.157341566, 0.047363349

x = [0, -foo0[1] * 100]
y = [0, foo0[0] * 100]

a = [0, -foo1[1] * 100]
b = [0, foo1[0] * 100]

# k = [0, -foo2[1] * 100]
# j = [0, foo2[0] * 10]

plt.xlim(-20, 20)
plt.ylim(-10, 10)

plt.plot(x, y, 'bo')
plt.plot(x, y, label="Max", color='red')

plt.plot(a, b, 'bo')
plt.plot(a, b, label="Average", color='green')

# plt.plot(k, j, 'bo')
# plt.plot(k, j, label="3rd Run", color='green')

plt.xlabel('X (cm)')
plt.ylabel('Y (cm)')
plt.title('IK Gaits')

plt.legend()
plt.show()
