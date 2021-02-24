import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# Show the part of transition probability being loaded to the solver

f = open("output/full_trans.csv","r") 
lines = f.readlines()

param = np.fromstring(lines[0], sep=',')[:-1]
lines = lines[1:]

N   = int(param[0]) # 1
n_x = int(param[1])
n_u = int(param[2])

# for i in range(n_x):
#     plt.subplot(2, 5, i+1).set_title(i)

# 2D grid map
mat = np.fromstring(lines[0], sep=',')[:-1]
mat = mat.reshape(n_x, n_u)
mat = mat[0:16*16,:]
print("done")

c = plt.pcolor(mat)
plt.colorbar(c)

# plt.xlim(xmin=0, xmax=n_p)

plt.show()
