import csv
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm
import copy

if __name__ == "__main__":
    # Show running cost. X-axis: acceleration. Y-axis: velocity.

    f = open("output/t_cost_dv.csv","r") 
    lines = f.readlines()

    # fig, (ax1, ax2) = plt.subplots(1, 2, gridspec_kw={'width_ratios': [2, 1]})
    fig, ax1 = plt.subplots()

    param = np.fromstring(lines[0], sep=',')[:-1]
    lines = lines[1:]

    N   = int(param[0])
    n_d = int(param[1])
    n_v = int(param[2])

    mat = np.fromstring(lines[0], sep=',')[:-1]

    # n, bins, patches = ax2.hist(mat, 100, density=False)

    # d = [-8.0, 2.0]
    # n_d = int(128)
    # d_list = []
    # for i in range(n_d):
    #     d_list.append(str('%.1f' % (d[0] + (d[1]-d[0])/(n_d-1)*i)))

    v = [.0, 18.0]
    n_v = int(16)
    v_list = []
    for i in range(n_v):
        v_list.append(str('%.1f' % (v[0] + (v[1]-v[0])/(n_v-1)*i)))

    print("%.3e, %.3e"%(mat.max(),mat.min()))

    # for i in range(len(mat)):
    #     if mat[i] < 1e-3 and mat[i] > -1e-3:
    #         mat[i] = np.nan
    # mat = np.ma.masked_invalid(mat)
    mat = mat.reshape(n_d, n_v)
    # mat1 = mat[... , :25]
    # mat2 = mat[... , 25:]

    my_cmap = copy.copy(cm.get_cmap("viridis"))
    my_cmap.set_bad(color='w',alpha = 1.)

    c = ax1.pcolormesh(mat, cmap=my_cmap)

    # a1_list = a_list[:25]
    # ax1.set_xticks(np.arange(len(a1_list)))
    # ax1.set_xticklabels(a1_list)
    # plt.setp(ax1.get_xticklabels(), rotation=90, ha="right",
    #      rotation_mode="anchor")
    # ax1.set_xlabel('acceleration')


    ax1.set_xticks(np.arange(len(v_list)))
    ax1.set_xticklabels(v_list)
    ax1.set_xlabel('velocity')

    fig.colorbar(c, ax=ax1)

    plt.show()
    
