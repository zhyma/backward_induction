import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# csvFile = open("data.csv","r")
# reader = csv.reader(csvFile)

mat = np.genfromtxt('prob.csv', delimiter=',')[:,:-1]
print(mat)
print(mat.shape)

for i in mat:
    if abs(sum(i)-1) > 0.000001:
        print(sum(i))

print("all checked")

shape = mat.shape
x,y=np.meshgrid(range(0, shape[1]), range(0, shape[0]))
fig = plt.figure('eigen_vector')
ax = fig.add_subplot(1, 1, 1, projection='3d')

ax.plot_surface(x,y,mat)
# ax.plot_wireframe(x,y,mat, alpha=0.5)
plt.show()