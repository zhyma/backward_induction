import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


f = open("raw_prob.csv","r") 
lines = f.readlines()

line = lines[1]
mat = np.fromstring(line, sep=',')[:-1]
print(mat)
print(mat.shape)
print(np.sum(mat))

# n, bins, patches = plt.hist(mat, 50, density=True, facecolor='g', alpha=0.75)
plt.plot(mat)
# plt.grid(True)
plt.show()