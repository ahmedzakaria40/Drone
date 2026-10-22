import pandas as pd
import matplotlib.pyplot as plt

d = pd.read_csv("traj.csv")
plt.plot(d.x,d.x)
plt.show()