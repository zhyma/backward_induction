import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm
import copy
import sys

def load(solver, out_type, n_w_cpu = -1):
    f = open('output/' + solver + '_' + out_type + '.csv','r') 
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

        if not n_w_cpu == -1:
            mat = np.delete(mat, list(range(n_w_cpu, n_w)), axis=1)
        
        result.append(mat)

    return N, n_x, n_w, result

def value(solver):
    if solver == 'cpu' or solver == 'gpu':
        print('solver type ' + solver)
        N, n_x, n_w, mats = load('121_46_31_' + solver, 'value')
    elif solver == 'compare':
        print('compare results')
        N, n_x, n_w, mats1 = load('241_46_31_cpu', 'value')
        _,   _,   _, mats2 = load('241_46_31_gpu', 'value', n_w)
        mats = []
        # for k in range(N-1, N):
        for k in range(N):
            print(k)
            diff_val = []
            max_val = -1
            mat = np.absolute(mats1[k] - mats2[k])

            mats.append(mat)
            for i in range(n_x):
                for j in range(n_w):
                    if (mats1[k][i,j] > 1e10) and (mats2[k][i,j] > 1e10):
                        mats[k][i,j] = 0
                    elif mats[k][i,j] < 1:
                        mats[k][i,j] = 0
                    else:
                        diff_val.append(mats[k][i,j])
                        if mats[k][i,j] > max_val:
                            max_val = mats[k][i,j]

            # print(diff_val)
            print(len(diff_val))
            print(max_val)

            print("====")

    for k in range(N):
        # k = 10 is the last step
        # 2D grid map
        fig, (ax1, ax2) = plt.subplots(1, 2, gridspec_kw={'width_ratios': [2, 1]}, figsize=(12,9))

        title = 'value at k=' + str(k)
        fig.suptitle(title)

        mat = mats[k]

        my_cmap = copy.copy(cm.get_cmap("rainbow"))
        # set_under() for vmin
        #  set_over() for vmax

        if solver == 'compare':
            my_cmap.set_under('w')
            # my_cmap.set_over('k')
            # c = ax1.pcolormesh(mat,cmap=my_cmap,vmin=0.5,vmax=max_val)
            c = ax1.pcolormesh(mat,cmap=my_cmap,vmin=0.5)
        else:
            m = mat[0,0]
            for i in range(n_x):
                for j in range(n_w):
                    if (mat[i,j] > m) and (mat[i,j] < 1e10):
                        m = mat[i,j]

            print(m)

            my_cmap.set_over('w')
            c = ax1.pcolormesh(mat,cmap=my_cmap,vmax=m)

        fig.colorbar(c, ax=ax1)

        ax1.set_xlabel('w(d*intention)')
        ax1.set_ylabel('x(d*v)')

        mat_histo = []
        for i in range(n_x):
            for j in range(n_w):
                if solver == 'compare':
                    if mat[i,j] > 0.5:
                        mat_histo.append(mat[i,j])
                else:
                    if mat[i, j] < 1e10:
                        mat_histo.append(mat[i,j])
        n, bins, patches = ax2.hist(mat_histo, 100, density=False)

        fig.tight_layout()
        plt.savefig('fig/' + solver + '_value_t_'+str(k)+'.png')
        # plt.show()

    print('done')

def action(solver):
    if solver == 'cpu' or solver == 'gpu':
        print('solver type ' + solver)
        N, n_x, n_w, mats = load('121_46_31_' + solver, 'action')

    for k in range(N):
        # k = 10 is the last step

        # 2D grid map
        fig, ax1 = plt.subplots(figsize=(12,9))

        title = 'action at k=' + str(k)
        fig.suptitle(title)

        mat = mats[k]

        my_cmap = copy.copy(cm.get_cmap("rainbow"))

        c = ax1.pcolormesh(mat,cmap=my_cmap, vmin=0, vmax=31)
        fig.colorbar(c, ax=ax1)

        ax1.set_xlabel('w(d*intention)')
        ax1.set_ylabel('x(d*v)')

        fig.tight_layout()
        plt.savefig('fig/' + solver + '_action_t_'+str(k)+'.png')

    print('done')


if __name__ == '__main__':

    if len(sys.argv) > 2:
        if sys.argv[1] == 'value':
            if sys.argv[2] == 'cpu':
                value('cpu')
            elif sys.argv[2] == 'gpu':
                value('gpu')
            elif sys.argv[2] == 'compare':
                value('compare')
        elif sys.argv[1] == 'action':
            if sys.argv[2] == 'cpu':
                action('cpu')
            elif sys.argv[2] == 'gpu':
                action('gpu')
            elif sys.argv[2] == 'compare':
                action('compare')
    else:
        print('parameters: value/action, cpu/gpu/compare')