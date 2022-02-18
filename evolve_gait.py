from deap import base
from deap import creator
from deap import tools
import random
import seaborn as sns
import numpy as np
import matplotlib.pyplot as plt
import hexapodengine as h
import elitism
import multiprocessing

DIMENSIONS = 3 + (h.SIZE_OF_MOTION_CHROMOSOME * h.MAX_MOTIONS_IN_SEQUENCE * h.NUM_OF_LEGS)
BOUNDS_LOW = [0.1, 50, 0] + ((([0] * 6) + ([-2] * 12)) * 4 * 6)
BOUNDS_HIGH = [12, 300, 1] + (([4] + [1] + ([1] * 4) + ([2] * 12)) * 4 * 6)

POPULATION_SIZE = 100
P_CROSSOVER = 0.9  # probability for crossover
P_MUTATION = 0.5  # (try also 0.5) probability for mutating an individual
MAX_GENERATIONS = 10
HALL_OF_FAME_SIZE = int(0.1 * POPULATION_SIZE)
CROWDING_FACTOR = 15.0  # crowding factor for crossover and mutation

toolbox = base.Toolbox()
creator.create("FitnessMax", base.Fitness, weights=(1.0, -1.0))
creator.create("Individual", list, fitness=creator.FitnessMax)

for i in range(DIMENSIONS):
    toolbox.register("layer_size_attribute_" + str(i), random.uniform, BOUNDS_LOW[i], BOUNDS_HIGH[i])

layer_size_attributes = ()
for i in range(DIMENSIONS):
    layer_size_attributes = layer_size_attributes + \
                            (toolbox.__getattribute__("layer_size_attribute_" + str(i)),)

toolbox.register("individualCreator", tools.initCycle, creator.Individual, layer_size_attributes, n=1)
toolbox.register("populationCreator", tools.initRepeat, list, toolbox.individualCreator)
toolbox.register("evaluate", h.evaluateGait)
toolbox.register("select", tools.selStochasticUniversalSampling)
toolbox.register("mate", tools.cxSimulatedBinaryBounded, low=BOUNDS_LOW, up=BOUNDS_HIGH, eta=CROWDING_FACTOR)
toolbox.register("mutate", tools.mutPolynomialBounded, low=BOUNDS_LOW, up=BOUNDS_HIGH, eta=CROWDING_FACTOR, indpb=0.03)


# Genetic Algorithm flow:
def main():
    pool = multiprocessing.Pool()
    toolbox.register("map", pool.map)
    population = toolbox.populationCreator(n=POPULATION_SIZE)
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("max", np.max)
    stats.register("avg", np.mean)
    hof = tools.HallOfFame(HALL_OF_FAME_SIZE)
    population, logbook = elitism.eaSimpleWithElitism(population, toolbox, cxpb=P_CROSSOVER, mutpb=P_MUTATION, ngen=MAX_GENERATIONS, stats=stats, halloffame=hof, verbose=True)
    best = hof.items[0]
    print("-- Best Individual = ", best)
    print("-- Best Fitness = ", best.fitness.values[0])
    maxFitnessValues, meanFitnessValues = logbook.select("max", "avg")
    pool.close()

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
