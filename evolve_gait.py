from deap import base
from deap import creator
from deap import tools
import random
import seaborn as sns
import matplotlib.pyplot as plt

DIMENSIONS = 3 + (SIZE_OF_MOTION_CHROMOSOME * MAX_MOTIONS_IN_SEQUENCE * NUM_OF_LEGS)  # number of dimensions
BOUNDS_LOW = [0.1, 50, 0] + ((([0] * 6) + ([-2] * 12)) * 4 * 6)
BOUNDS_HIGH = [12, 300, 1] + (([4] + [1] + ([1] * 4) + ([2] * 12)) * 4 * 6)

POPULATION_SIZE = 100
P_CROSSOVER = 0.9  # probability for crossover
P_MUTATION = 0.5  # (try also 0.5) probability for mutating an individual
MAX_GENERATIONS = 100
HALL_OF_FAME_SIZE = int((1. / 10.) * POPULATION_SIZE)
CROWDING_FACTOR = 15.0  # crowding factor for crossover and mutation

toolbox = base.Toolbox()
creator.create("FitnessMax", base.Fitness, weights=(1.0, -0.5))
creator.create("Individual", list, fitness=creator.FitnessMax)

for i in range(DIMENSIONS):
    toolbox.register("layer_size_attribute_" + str(i), random.uniform, BOUNDS_LOW[i], BOUNDS_HIGH[i])

layer_size_attributes = ()
for i in range(DIMENSIONS):
    layer_size_attributes = layer_size_attributes + \
                            (toolbox.__getattribute__("layer_size_attribute_" + str(i)),)

toolbox.register("individualCreator", tools.initCycle, creator.Individual, layer_size_attributes, n=1)
toolbox.register("populationCreator", tools.initRepeat, list, toolbox.individualCreator)
toolbox.register("evaluate", evaluateGait)
toolbox.register("select", tools.selTournament, tournsize=2)
toolbox.register("mate", tools.cxSimulatedBinaryBounded, low=BOUNDS_LOW, up=BOUNDS_HIGH, eta=CROWDING_FACTOR)
toolbox.register("mutate", tools.mutPolynomialBounded, low=BOUNDS_LOW, up=BOUNDS_HIGH, eta=CROWDING_FACTOR, indpb=2.0 / 100.0)


# Genetic Algorithm flow:
def main():
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
