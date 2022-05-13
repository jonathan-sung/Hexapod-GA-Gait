import math
from matplotlib import pyplot as plt
import pandas as pd
import numpy as np
import scipy.stats as stats
import math


def BlendCX(parent1, parent2, a):
    lower = parent1 - a * (parent2 - parent1)
    upper = parent2 + a * (parent2 - parent1)
    return lower, upper


def SBX(parent1, parent2, b):
    lower = (1 / 2) * ((1 + b) * parent1 + (1 - b) * parent2)
    upper = (1 / 2) * ((1 - b) * parent1 + (1 + b) * parent2)
    return lower, upper


data_dict = {}
a = 1.33
b = 5.72
# data_dict['category'] = ['α = 0', 'α = 0.5', 'α = 1']
# data_dict['parent_lower'] = [BlendCX(a, b, 0)[0], BlendCX(a, b, 0)[0], BlendCX(a, b, 0)[0]]
# data_dict['parent_higher'] = [BlendCX(a, b, 0)[1], BlendCX(a, b, 0)[1], BlendCX(a, b, 0)[1]]
# data_dict['lower'] = [BlendCX(a, b, 0)[0], BlendCX(a, b, 0.5)[0], BlendCX(a, b, 1)[0]]
# data_dict['upper'] = [BlendCX(a, b, 0)[1], BlendCX(a, b, 0.5)[1], BlendCX(a, b, 1)[1]]

data_dict['category'] = ['β = 0.8', 'β = 1.0', 'β = 1.2']
data_dict['parent_lower'] = [SBX(a, b, 1)[0], SBX(a, b, 1)[0], SBX(a, b, 1)[0]]
data_dict['parent_higher'] = [SBX(a, b, 1)[1], SBX(a, b, 1)[1], SBX(a, b, 1)[1]]
data_dict['lower'] = [SBX(a, b, 0.8)[0], SBX(a, b, 1)[0], SBX(a, b, 1.2)[0]]
data_dict['upper'] = [SBX(a, b, 0.8)[1], SBX(a, b, 1)[1], SBX(a, b, 1.2)[1]]
dataset = pd.DataFrame(data_dict)

# for lower, upper, parent_lower, parent_higher, y in zip(dataset['lower'], dataset['upper'], dataset['parent_lower'], dataset['parent_higher'], range(len(dataset))):
#     plt.plot((lower, upper, parent_lower, parent_higher), (y, y, y, y), 'ro-', color='#2187bb')
# plt.yticks(range(len(dataset)), list(dataset['category']))

mu = 0
variance = 1
sigma = math.sqrt(variance)
x = np.linspace(mu - 3*sigma, mu + 3*sigma, 100)
y = x * 0
plt.plot(x, stats.norm.pdf(x, mu, sigma))

plt.show()

