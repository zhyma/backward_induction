import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# csvFile = open("data.csv","r")
# reader = csv.reader(csvFile)

mat = np.genfromtxt('gaussian_test.csv', delimiter=',')[:-1]
print(mat)
print(mat.shape)

n, bins, patches = plt.hist(mat, 50, density=True, facecolor='g', alpha=0.75)

plt.grid(True)
plt.show()