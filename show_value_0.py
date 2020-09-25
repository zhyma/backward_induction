import csv
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import xml.etree.ElementTree as ET

# Wasted.
# only take out w = 0 for test. Realized that it doesn't work

tree = ET.parse('config.xml')
root = tree.getroot()
gran = 10
for child in root:
    if child.tag == 'granularity':
        gran = int(child.text)

mat1 = np.genfromtxt('value.csv', delimiter=',')[:,:-1]
print(mat1.shape)

with open('value.csv','r') as csvfile:
    reader = csv.reader(csvfile)
    row0 = next(reader)

marker = []
title = []
for idx, val in enumerate(row0[:-1]):
    w = float(val.split(';')[1])
    if w == 0:
        marker.append(idx)
        title.append(val.split(';')[0])

print(title)
print(marker)

# x from -2, 2. N=10
# delete element, (mat1, column/row No. 0, axis=0/row-wise)
mat0 = np.delete(mat1, 0, 0)
mat1 = np.zeros((mat0.shape[0], len(marker)))
print(mat1.shape)
for i in range(mat1.shape[0]):
    for j in range(mat1.shape[1]):
        mat1[i, j] = mat0[i, marker[j]]
        
x,y=np.meshgrid(range(mat1.shape[1]), range(mat1.shape[0]))
fig = plt.figure('value table and action table')
ax1 = fig.add_subplot(2, 1, 1, projection='3d')
ax1.set_xlabel('\n system state x(k)')
ax1.set_ylabel('\n No. k step')
ax1.set_zlabel('\n Value (the lower the better)')
# ax1.set_xticks(np.arange(-2, 2.1, 0.5))
ax1.set_yticks(range(0, mat1.shape[0], 1))

ax1.plot_surface(x, y, mat1, cmap=cm.coolwarm)
# ax.plot_wireframe(x,y,mat, alpha=0.5)

mat2 = np.genfromtxt('action.csv', delimiter=',')[:,:-1]
# delete element, (mat1, column/row No. 0, axis=0/row-wise)
mat2 = np.delete(mat2, 0, 0)
ax2 = fig.add_subplot(2, 1, 2, projection='3d')
x,y=np.meshgrid(range(mat2.shape[1]), range(mat2.shape[0]))
ax2.plot_surface(x,y,mat2, cmap=cm.coolwarm)
ax2.set_xlabel('\n system state x(k)')
ax2.set_ylabel('\n No. k step')
ax2.set_zlabel('\n best u')
# ax2.set_xticks(np.arange(-2, 2.1, 0.5))
ax2.set_yticks(range(0, mat2.shape[0], 1))
ax2.set_zticks(np.arange(0, 2, 0.4))

plt.show()