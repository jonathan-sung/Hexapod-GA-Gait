import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("C:/Users/Jonathan/Desktop/output_6000.csv", index_col=0)
df.plot()
plt.title("Average Evaluation Metrics over Generations")
plt.xlabel("Generation")
plt.ylabel("Fitness")
plt.show()
