import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# Show the part of transition probability being loaded to the solver

f = open("output/full_prob.csv","r") 
lines = f.readlines()

param = np.fromstring(lines[0], sep=',')[:-1]
lines = lines[1:]

N   = int(param[0])
n_w = int(param[1])
n_p = int(param[2])

for i in range(N):
    plt.subplot(2, (N+1)//2, i+1).set_title(i)

    mat = np.fromstring(lines[i], sep=',')[:-1]
    mat = mat.reshape(n_w, n_p)

    near = 0
    far = n_w
    
    for j in range(n_w):
        if sum(mat[j]) == 0:
            near += 1
        else:
            break
    # for j in range(n_w-1, -1, -1):
    #     if sum(mat[j]) == 0:
    #         far -= 1
    #     else:
    #         break

    # mat = mat[near:near+256,:]
    mat = mat[0:512,:]
    c = plt.pcolor(mat)
    plt.colorbar(c)

    plt.xlim(xmin=0, xmax=n_p)
    # plt.ylim(ymin=0, ymax=n_w)

plt.show()
