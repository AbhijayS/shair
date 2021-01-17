import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import numpy as np

df = pd.read_csv('calibration.txt', names=['x','y','z'], delim_whitespace=True)
x = df['x'].to_numpy()
y = df['y'].to_numpy()
z = df['z'].to_numpy()

# order: x y z
MAG_HARD_IRON = np.array([-1251.140526, 121.490963, 3250.619550])
# matrix copied from magneto
MAG_SOFT_IRON = np.array([
    [1.028982, 0.009700, -0.014950],
    [0.009700, 1.125870, -0.000530],
    [-0.014950, -0.000530, 1.013084]])

fig = plt.figure()
ax0 = fig.add_subplot(211, projection='3d')
ax0.scatter(x,y,z)

cx = x-MAG_HARD_IRON[0]
cy = y-MAG_HARD_IRON[1]
cz = z-MAG_HARD_IRON[2]
calibrated = MAG_SOFT_IRON.dot(np.stack((cx,cy,cz)))

ax1 = fig.add_subplot(212, projection='3d')
ax1.scatter(calibrated[0],calibrated[1],calibrated[2])

# ax1 = fig.add_subplot(222)
# ax1.scatter(df['x'].to_numpy(), df['z'].to_numpy())

# ax2 = fig.add_subplot(223)
# ax2.scatter(df['y'].to_numpy(), df['z'].to_numpy())

# ax3 = fig.add_subplot(224)
# ax3.scatter(df['x'].to_numpy(), df['y'].to_numpy())

plt.show()