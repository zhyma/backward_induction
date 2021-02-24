import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm
import copy
import sys

def value(mode):
    f = open('output/' + mode + '_value.csv','r') 
    lines = f.readlines()

    dim = np.fromstring(lines[0], sep=',')
    print(dim)
    N   = int(dim[0])
    n_x = int(dim[1])
    n_w = int(dim[2])

    for t in range(11):
        # t = 10 is the last step
        # t = 9
        i = t+1

        # 2D grid map
        fig, (ax1, ax2) = plt.subplots(1, 2, gridspec_kw={'width_ratios': [2, 1]})

        title = 'value at k=' + str(t)
        fig.suptitle(title)

        mat = np.fromstring(lines[i], sep=',')[:-1]
        mat_histo = np.copy(mat)
        # max
        m = 0
        for i in range(mat.shape[0]):
            if (mat[i] > m) and (mat[i] < 10e15):
                m = mat[i]
        # for i in range(mat.shape[0]):
        #     if (mat[i] > m):
        #         mat[i] = m+100
        mat = mat.reshape(n_x,n_w)
        # print(mat.shape)
        if mode == 'gpu':
            mat = np.delete(mat, list(range(256, 260)), axis=1)
            print(mat.shape)
        print(m)

        my_cmap = copy.copy(cm.get_cmap("rainbow"))
        # set_under() for vmin
        my_cmap.set_over('w')

        c = ax1.pcolormesh(mat,cmap=my_cmap,vmax=m)
        fig.colorbar(c, ax=ax1)

        # plt.xlim(xmin=0, xmax=n_w)
        # plt.ylim(ymin=0, ymax=n_x)

        # mat_histo = np.fromstring(lines[i], sep=',')[:-1]
        for j in range(mat_histo.shape[0]-1, -1, -1):
            if mat_histo[j] > m:
                mat_histo = np.delete(mat_histo, j)
        n, bins, patches = ax2.hist(mat_histo, 100, density=False)
        # ax2.set_yscale('log',base=10)

        fig.tight_layout()
        # plt.show()
        plt.savefig('fig/' + mode + '_value_t_'+str(t)+'.png')

    print('done')


if __name__ == '__main__':

    if len(sys.argv) > 1:
        if sys.argv[1] == 'cpu':
            value('cpu')
        elif sys.argv[1] == 'gpu':
            value('gpu')
        elif sys.argv[1] == 'both':
            value('cpu')
            value('gpu')
    else:
        print('Default: cpu only')
        process('cpu')
            
    