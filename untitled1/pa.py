import pandas as pd
import numpy as np
from pandas import DataFrame, Series
import matplotlib.pyplot as plt

trpo=pd.read_csv("trpo.csv")
ddpg=pd.read_csv("ddpg.csv")

#new=pd.DataFrame([trpo.AverageReturn,ddpg.AverageReturn])
t=np.arange(0,600,1)

fig = plt.figure()
plt.plot(t,trpo.AverageReturn,label="trpo")
plt.plot(t,ddpg.AverageReturn,label="ddpg")
plt.legend(bbox_to_anchor=(0.9, 0.2), loc="best")
plt.show()
