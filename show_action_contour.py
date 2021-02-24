import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm
import copy


f = open("output/cpu_action.csv","r") 
lines = f.readlines()

dim = np.fromstring(lines[0], sep=',')
print(dim)
N   = int(dim[0])
n_x = int(dim[1])
n_w = int(dim[2])

for t in range(10):
    # t = 9 is the last step
    # t = 9
    i = t+1

    # 2D grid map
    fig, (ax1, ax2) = plt.subplots(1, 2, gridspec_kw={'width_ratios': [2, 1]})

    title = 'optimal control at k=' + str(t)
    fig.suptitle(title)

    mat = np.fromstring(lines[i], sep=',')[:-1]
    mat_histo = np.copy(mat)

    mat = mat.reshape(n_x,n_w)

    my_cmap = copy.copy(cm.get_cmap("rainbow"))
    my_cmap.set_under('w')

    c = ax1.pcolormesh(mat,cmap=my_cmap, vmin=0, vmax=31)
    fig.colorbar(c, ax=ax1)

    n, bins, patches = ax2.hist(mat_histo, 32, density=False)
    ax2.set_xlim(xmin=0, xmax=31)
    ax2.set_yscale('log',base=10)

    fig.tight_layout()
    # plt.show()
    plt.savefig('fig/action_t_'+str(t)+'.png')
