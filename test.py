import random
import math

l1 = [x for x in list(range(0, 24)) if x not in list(range(3, 24, 4))]
l2 = [x for x in range(0, 24) if (x + 1) % 4 != 0]
print(l1)
print(len(l1))
print(l2)
print(len(l2))
cheese = random.uniform(0, 0)
print(cheese)

x = float('nan')
print(math.isnan(x))
print(x)

