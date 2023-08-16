import matplotlib.pyplot as plt
import numpy as np
with open("cooperPath.txt", 'r') as f:
    f.readline()
    lin_vels = f.readline().strip().replace("[", "").replace("]", "").split(", ")
    ang_vels = f.readline().strip().replace("[", "").replace("]", "").split(", ")

lin_vels = [float(i) for i in lin_vels]
ang_vels = [float(i) for i in ang_vels]
plt.plot(lin_vels, '.')
plt.plot(ang_vels, '.')
plt.show()