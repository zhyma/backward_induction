import csv
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.cm as cm
import copy

if __name__ == "__main__":
    # Show running cost. X-axis: acceleration. Y-axis: velocity.

    f = open("output/r_cost_va.csv","r") 
    lines = f.readlines()

    fig, (ax1, ax2) = plt.subplots(1, 2, gridspec_kw={'width_ratios': [2, 1]})

    param = np.fromstring(lines[0], sep=',')[:-1]
    lines = lines[1:]

    N   = int(param[0])
    n_v = int(param[1])
    n_a = int(param[2])

    mat = np.fromstring(lines[0], sep=',')[:-1]

    # n, bins, patches = ax2.hist(mat, 100, density=False)

    a = [-8.0, 2.0]
    n_a = int(32)
    a_list = []
    for i in range(n_a):
        a_list.append(str('%.1f' % (a[0] + (a[1]-a[0])/(n_a-1)*i)))

    v = [.0, 18.0]
    n_v = int(16)
    v_list = []
    for i in range(n_v):
        v_list.append(str('%.1f' % (v[0] + (v[1]-v[0])/(n_v-1)*i)))

    print("%.3e, %.3e"%(mat.max(),mat.min()))

    for i in range(len(mat)):
        if mat[i] < 1e-3 and mat[i] > -1e-3:
            mat[i] = np.nan
    mat = np.ma.masked_invalid(mat)
    mat = mat.reshape(n_v, n_a)
    mat1 = mat[... , :25]
    mat2 = mat[... , 25:]

    my_cmap = copy.copy(cm.get_cmap("viridis"))
    my_cmap.set_bad(color='w',alpha = 1.)

    c1 = ax1.pcolormesh(mat1, cmap=my_cmap)
    c2 = ax2.pcolormesh(mat2, cmap=my_cmap)

    a1_list = a_list[:25]
    ax1.set_xticks(np.arange(len(a1_list)))
    ax1.set_xticklabels(a1_list)
    plt.setp(ax1.get_xticklabels(), rotation=90, ha="right",
         rotation_mode="anchor")
    ax1.set_xlabel('acceleration')

    a2_list = a_list[25:]
    ax2.set_xticks(np.arange(len(a2_list)))
    ax2.set_xticklabels(a2_list)
    plt.setp(ax2.get_xticklabels(), rotation=90, ha="right",
         rotation_mode="anchor")
    ax2.set_xlabel('acceleration')

    ax1.set_yticks(np.arange(len(v_list)))
    ax1.set_yticklabels(v_list)
    ax1.set_ylabel('velocity')

    ax2.set_yticks(np.arange(len(v_list)))
    ax2.set_yticklabels(v_list)
    ax2.set_ylabel('velocity')

    fig.colorbar(c1, ax=ax1)
    fig.colorbar(c2, ax=ax2)

    plt.show()
    
