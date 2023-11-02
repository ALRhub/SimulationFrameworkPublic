import matplotlib.pyplot as plt
import numpy as np

np.random.seed(42)

degrees = np.random.random_sample(60) * 180 - 90
x_pos = np.random.random_sample(60) * 0.2 + 0.3
y_pos = np.random.random_sample(60) * 0.5 - 0.35

fig = plt.figure()
ax = fig.add_subplot(projection="3d")
ax.scatter(x_pos, y_pos, degrees)
plt.show()
