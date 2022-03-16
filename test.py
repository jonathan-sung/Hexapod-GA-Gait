import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import re


def resetCounters():
    global currentLineCount
    global currentTotal
    currentLineCount = 0
    currentTotal = [0] * NUM_OF_METRICS

NUM_OF_METRICS = 6
currentLineCount = 0
currentTotal = [0] * NUM_OF_METRICS
listOfAvgFitness = []
currentGen = 0

with open("C:/Users/Jonathan/Desktop/tripod_cyclic_log_1.txt") as infile:
    for line in infile:
        x = re.search("\((\d+\.\d+, ){" + str(NUM_OF_METRICS - 1) + "}\d+\.\d+\)", line)
        y = re.search("^\d+\s+", line)
        if x:
            raw_str = (x.group(0)[1:-1]).replace(" ", "")
            fitnessList = [float(i) for i in raw_str.split(",")]
            currentTotal = [j + k for j, k in zip(currentTotal, fitnessList)]
            currentLineCount += 1
        elif y:
            avgFitness = [j / k for j, k in zip(currentTotal, [currentLineCount] * NUM_OF_METRICS)]
            print(currentGen, avgFitness)
            listOfAvgFitness.append(avgFitness)
            resetCounters()
            currentGen += 1
    df = pd.DataFrame(listOfAvgFitness, columns=["Distance", "Angle", "Stability", "Height", "Leg Collision", "Avg Fitness"])
    df.to_csv("C:/Users/Jonathan/Desktop/output_tripod.csv")
    print(df)
plt.show()
