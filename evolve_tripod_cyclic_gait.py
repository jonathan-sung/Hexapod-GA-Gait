from deap import base
from deap import creator
from deap import tools
from deap import algorithms
import random
import seaborn as sns
import numpy as np
import matplotlib.pyplot as plt
import hexapodengine4 as h
import elitism
import multiprocessing
import math
import time

DIMENSIONS = 2 + h.LENGTH_OF_GAIT_STATE * (h.LENGTH_OF_SEQUENCE - 1)
BOUNDS_LOW = [0.2] + [140] + ([0.2] + ([-h.servoRangeOfMotion] * h.NUM_OF_SERVOS)) * (h.LENGTH_OF_SEQUENCE - 1)
BOUNDS_HIGH = [1] + [160] + ([1] + ([h.servoRangeOfMotion] * h.NUM_OF_SERVOS)) * (h.LENGTH_OF_SEQUENCE - 1)

POPULATION_SIZE = 6000
P_CROSSOVER = 0.9  # probability for crossover
P_MUTATION = 0.5  # probability for mutating an individual
MAX_GENERATIONS = 1000
HALL_OF_FAME_SIZE = int(0.1 * POPULATION_SIZE)
CROWDING_FACTOR = 20.0  # crowding factor for crossover and mutation
TOURN_SIZE = 2
MUT_INDPB = 1.0 / DIMENSIONS
DISTANCE_WEIGHT = 1.0
ANGLE_WEIGHT = -1.0
STABILITY_WEIGHT = 1.0
HEIGHT_WEIGHT = -1.0

estimatedTime = (0.115 * POPULATION_SIZE * MAX_GENERATIONS) / (60.0 * 60.0)
print(f"Estimated Evolution Time: {estimatedTime} hours")
print("GA Parameters", POPULATION_SIZE, P_CROSSOVER, P_MUTATION, MAX_GENERATIONS, HALL_OF_FAME_SIZE, CROWDING_FACTOR, DIMENSIONS, TOURN_SIZE, MUT_INDPB, DISTANCE_WEIGHT, ANGLE_WEIGHT, STABILITY_WEIGHT)

toolbox = base.Toolbox()
creator.create("FitnessCompound", base.Fitness, weights=(1.0,))
creator.create("Individual", list, fitness=creator.FitnessCompound)

# creator.create("FitnessSingle", base.Fitness, weights=(DISTANCE_WEIGHT,))
# creator.create("Individual", list, fitness=creator.FitnessSingle)

for i in range(DIMENSIONS):
    toolbox.register("layer_size_attribute_" + str(i), random.uniform, BOUNDS_LOW[i], BOUNDS_HIGH[i])

layer_size_attributes = ()
for i in range(DIMENSIONS):
    layer_size_attributes = layer_size_attributes + \
                            (toolbox.__getattribute__("layer_size_attribute_" + str(i)),)

toolbox.register("individualCreator", tools.initCycle, creator.Individual, layer_size_attributes, n=1)
toolbox.register("populationCreator", tools.initRepeat, list, toolbox.individualCreator)
toolbox.register("evaluate", h.evaluateGait)
toolbox.register("select", tools.selTournament, tournsize=TOURN_SIZE)
# toolbox.register("select", tools.selNSGA2)
toolbox.register("mate", tools.cxSimulatedBinaryBounded, low=BOUNDS_LOW, up=BOUNDS_HIGH, eta=CROWDING_FACTOR)
toolbox.register("mutate", tools.mutPolynomialBounded, low=BOUNDS_LOW, up=BOUNDS_HIGH, eta=CROWDING_FACTOR, indpb=MUT_INDPB)


# Genetic Algorithm flow:
def main():
    lastTime = time.time()
    pool = multiprocessing.Pool()
    toolbox.register("map", pool.map)
    population = toolbox.populationCreator(n=POPULATION_SIZE)
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("max", np.max)
    stats.register("avg", np.mean)
    hof = tools.HallOfFame(HALL_OF_FAME_SIZE)
    # hof = tools.ParetoFront()
    population, logbook = elitism.eaSimpleWithElitism(population, toolbox, cxpb=P_CROSSOVER, mutpb=P_MUTATION, ngen=MAX_GENERATIONS, stats=stats, halloffame=hof, verbose=True)
    # population, logbook = algorithms.eaSimple(population, toolbox, cxpb=P_CROSSOVER, mutpb=P_MUTATION, ngen=MAX_GENERATIONS, stats=stats, halloffame=hof, verbose=True)
    best = hof.items[0]
    print("-- Best Individual = ", best)
    print("-- Best Fitness = ", best.fitness.values[0])
    maxFitnessValues, meanFitnessValues = logbook.select("max", "avg")
    pool.close()
    print(f"Total Time Elapsed: {((time.time() - lastTime) / 60.0)} minutes")

    # plot statistics:
    sns.set_style("whitegrid")
    plt.plot(maxFitnessValues, color='red')
    plt.plot(meanFitnessValues, color='green')
    plt.xlabel('Generation')
    plt.ylabel('Max / Average Fitness')
    plt.title('Max and Average fitness over Generations')
    plt.show()


if __name__ == "__main__":
    main()
