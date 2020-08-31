import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# csvFile = open("data.csv","r")
# reader = csv.reader(csvFile)

mat1 = np.genfromtxt('value.csv', delimiter=',')[:,:-1]

# x from -2, 2. N=10
x,y=np.meshgrid(np.arange(-2, 2.1, 0.1), range(10))
fig = plt.figure('eigen_vector')
ax1 = fig.add_subplot(2, 1, 1, projection='3d')

ax1.plot_surface(x,y,mat1)
# ax.plot_wireframe(x,y,mat, alpha=0.5)

mat2 = np.genfromtxt('action.csv', delimiter=',')[:,:-1]

ax2 = fig.add_subplot(2, 1, 2, projection='3d')
ax2.plot_surface(x,y,mat2)

plt.show()