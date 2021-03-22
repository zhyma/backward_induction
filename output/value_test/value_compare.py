import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm
import copy
import sys

def load(filename):
    f = open(filename + '.csv','r') 
    lines = f.readlines()

    dim = np.fromstring(lines[0], sep=',')
    N   = int(dim[0])
    n_x = int(dim[1])
    n_w = int(dim[2])

    result = []

    for t in range(N):
        i = t+1

        mat = np.fromstring(lines[i], sep=',')[:-1]
        # mat = mat.astype(int)
        mat = mat.reshape(n_x,n_w)

        result.append(mat)

    return N, n_x, n_w, result

if __name__ == '__main__':
    N, n_x, n_w, value7 = load('cpu_value_7')
    _, _, _, value8 = load('cpu_value_8')
    

    # print(N)
    for i in range(1,N+1):
        diff = (value7[-i]-value8[-i]).flatten()
        filtered = []
        for d in diff:
            if d < 1e13:
                filtered.append(d)

        fed = np.array(filtered)
        
        print("%d, max = %d, min = %d"%(fed.max()-fed.min(),fed.max(),fed.min()))
