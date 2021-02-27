import matplotlib.pyplot as plt
import numpy as np
import sys

def get_data(mode, t):
    f = open('output/' + mode + '_value.csv','r') 
    lines = f.readlines()

    dim = np.fromstring(lines[0], sep=',')
    N   = int(dim[0])
    n_x = int(dim[1])
    n_w = int(dim[2])

    mat = np.fromstring(lines[t+1], sep=',')[:-1]

    new_mat = []
    for i in range(len(mat)):
        if mat[i] < 1e29:
            new_mat.append(mat[i])

    print(len(new_mat))
    return new_mat

def histo(k, save):
    solver = 'gpu'
    data = get_data(solver, k)
    # plt.xscale('log')
    plt.yscale('log')
    plt.hist(data,bins=100)
    if save:
        plt.savefig('fig/' + solver + '_valuehist_t_'+str(k)+'.png')
        plt.clf()
    else:
        plt.show()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1].isdigit():
            histo(int(sys.argv[1]), False)
    else:
        for i in range(11):
            histo(i, True)