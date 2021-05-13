import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import sys

if __name__ == '__main__':

    # e.g.: cpu_prob_part.csv
    # e.g.: prob_full

    solver = '' # cpu/gpu
    param  = '' # tran/prob
    size   = '' # full/part
    filename = ''
    if len(sys.argv) == 4:
        solver = sys.argv[1]
        param  = sys.argv[2]
        size   = sys.argv[3]
        filename = 'output/' + solver + '_' + param + '_' + size + '.csv'
    elif len(sys.argv) ==3:
        param  = sys.argv[1]
        size   = sys.argv[2]
        filename = 'output/' + param + '_' + size + '.csv'
    else:
        print('argument error')

    f = open(filename,'r')
    lines = f.readlines()

    var = np.fromstring(lines[0], sep=',')[:-1]
    lines = lines[1:]

    if param == 'prob':
        N   = int(var[0])
        n_w = int(var[1])
        n_p = int(var[2])
        for i in range(N):
            fig, ax = plt.subplots(figsize=(12,9))

            # 2D grid map
            mat = np.fromstring(lines[i], sep=',')[:-1]
            mat = mat.reshape(n_w, n_p)

            if size == 'full':
                mat = mat[0:512,:]

            c = ax.pcolormesh(mat)
            fig.colorbar(c)

            if solver == '':
                plt.savefig('fig/' + param +\
                            '_' + size + '_' + str(i)+'.png')
            else:
                plt.savefig('fig/' + solver + '_' + param +\
                        '_' + size + '_' + str(i)+'.png')


    if param == 'tran':
        N   = int(var[0]) # 1
        n_x = int(var[1])
        n_u = int(var[2])

        # for i in range(n_x):
        #     plt.subplot(2, 5, i+1).set_title(i)

        # 2D grid map
        mat = np.fromstring(lines[0], sep=',')[:-1]
        mat = mat.reshape(n_x, n_u)
        mat = mat[0:16*16,:]

        fig, ax = plt.subplots(figsize=(12,9))

        c = ax.pcolormesh(mat)
        fig.colorbar(c)

        if solver == '':
                plt.savefig('fig/' + param +\
                            '_' + size + '.png')
        else:
            plt.savefig('fig/' + solver + '_' + param +\
                    '_' + size +'.png')
