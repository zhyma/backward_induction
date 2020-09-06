import csv
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import xml.etree.ElementTree as ET

mat1 = np.genfromtxt('mc_test.csv', delimiter=',')[:,:-1]
print(mat1.shape)

# x from -2, 2. N=10
x,y=np.meshgrid(range(mat1.shape[1]), range(mat1.shape[0]))
fig = plt.figure('eigen_vector')
ax = fig.add_subplot(2, 1, 1, projection='3d')
ax.set_xlabel('\n system state x(k)')
ax.set_ylabel('\n No. k step')
ax.set_zlabel('\n Value (the lower the better)')
# ax.set_xticks(np.arange(-2, 2.1, 0.5))
# ax.set_yticks(range(0, mat1.shape[0], 2))

ax.plot_surface(x, y, mat1, cmap=cm.coolwarm)
# ax.plot_wireframe(x,y,mat, alpha=0.5)

plt.show()