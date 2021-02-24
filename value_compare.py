import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm
import copy
import sys

def get_data(mode, t):
    f = open('output/' + mode + '_value.csv','r') 
    lines = f.readlines()

    dim = np.fromstring(lines[0], sep=',')
    print(dim)
    N   = int(dim[0])
    n_x = int(dim[1])
    n_w = int(dim[2])

    mat = np.fromstring(lines[t+1], sep=',')[:-1]

    mat = mat.reshape(n_x,n_w)
    if mode == 'gpu':
        mat = np.delete(mat, list(range(256, 260)), axis=1)

    return N, n_x, n_w, mat

def compare(t):
    N, n_x, n_w, cpu_mat = get_data('cpu', t)
    _, _, _, gpu_mat = get_data('gpu', t)
    print(cpu_mat.shape)
    print(gpu_mat.shape)
    print(np.array_equal(cpu_mat, gpu_mat))
    
    error = 0
    same = 0
    error_mat = []
    max = 0
    max_idx = []
    cpu_dist = []
    for i in range(n_x):
        for j in range(n_w):
            diff = abs(cpu_mat[i, j] - gpu_mat[i, j])
            if cpu_mat[i, j] > 1e15 and gpu_mat[i, j] > 1e15:
                continue
            # elif cpu_mat[i, j] > 1e18 or gpu_mat[i, j] < 1e18:
            #     # print("at %d, %d, cpu hits the constraint" %(i, j))
            #     cpu_c += 1
            # elif gpu_mat[i, j] > 1e18 and cpu_mat[i, j] < 1e18:
            #     # print("at %d, %d, gpu hits the constraint" %(i, j))
            #     gpu_c += 1
            #     cpu_dist.append(cpu_mat[i,j])
            elif diff > 0:
                # print("at %d, %d, error is %f" %(i, j, diff))
                diff_precent = diff/cpu_mat[i,j]
                if diff/gpu_mat[i,j] > diff/cpu_mat[i,j]:
                    diff_precent = diff/gpu_mat[i,j]
                if diff_precent > max:
                    max = diff_precent
                    max_idx = [i,j]
                if diff_precent > 0.01:
                    error_mat.append(diff)
                    error += 1
                    
            else:
                same += 1

    print('total error: %d, same: %d'\
        %(error, same))

    print(same)
    print(max, end='@')
    print(max_idx)
    # plt.hist(diff_precent, density=False, bins=10)
    # plt.show()
    return max, max_idx


if __name__ == '__main__':

    if len(sys.argv) > 1:
        if sys.argv[1].isdigit():
            print(sys.argv[1])
            compare(int(sys.argv[1]))
        elif sys.argv[1] == 'all':
            max_all = 0
            max_idx_all = []
            k_idx = 0
            for i in range(11):
                max_k, max_idx_k = compare(i)
                if max_k > max_all:
                    max_all = max_k
                    max_idx_all = max_idx_k
                    k_idx = i
            
            print('max error at step %d, percentage %f, at [%d, %d]'%(k_idx, max_all, max_idx_all[0], max_idx_all[1]))

    