import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import sys

# Show the part of transition probability being loaded to the solver

filename = 'output/partial_prob_'

if len(sys.argv) > 1:
    if sys.argv[1] == 'cpu':
        filename += 'cpu'
    elif sys.argv[1] == 'gpu':
        filename += 'gpu'
else:
    filename += 'cpu'

f = open(filename+'.csv',"r")
lines = f.readlines()

param = np.fromstring(lines[0], sep=',')[:-1]
lines = lines[1:]

N   = int(param[0])
n_w = int(param[1])
n_p = int(param[2])

for i in range(N):
    plt.subplot(2, 5, i+1).set_title(i)

    # 2D grid map
    mat = np.fromstring(lines[i], sep=',')[:-1]
    mat = mat.reshape(n_w, n_p)

    c = plt.pcolor(mat)
    plt.colorbar(c)

    plt.xlim(xmin=0, xmax=n_p)

plt.show()
