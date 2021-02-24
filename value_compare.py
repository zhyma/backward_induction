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
    N   = int(dim[0])
    n_x = int(dim[1])
    n_w = int(dim[2])

    mat = np.fromstring(lines[t+1], sep=',')[:-1]

    mat = mat.reshape(n_x,n_w)
    if mode == 'gpu':
        mat = np.delete(mat, list(range(256, 260)), axis=1)

    return N, n_x, n_w, mat

def compare(t):
    print("\nk=%d"%(t))
    N, n_x, n_w, cpu_mat = get_data('cpu', t)
    _, _, _, gpu_mat = get_data('gpu', t)
    # print(cpu_mat.shape)
    # print(gpu_mat.shape)
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
            if cpu_mat[i, j] > 1e29 and gpu_mat[i, j] > 1e29:
                continue
            elif diff > 0:
                # print("at %d, %d, error is %f" %(i, j, diff))
                diff_precent = abs(diff/cpu_mat[i,j])
                if abs(diff/gpu_mat[i,j]) > abs(diff/cpu_mat[i,j]):
                    diff_precent = abs(diff/gpu_mat[i,j])
                if diff_precent > max:
                    max = diff_precent
                    max_idx = [i,j]
                if diff_precent > 0.001:
                    error_mat.append(diff)
                    error += 1
                if diff > 0.1:
                    if diff < 1e5:
                        print("cpu_value=%.3f, diff_value=%.3f, percentage=%.6f"%(cpu_mat[i, j],diff,diff_precent*100))
                    else:
                        print("cpu_value=%.2e, diff_value=%.2e, percentage=%.6f"%(cpu_mat[i, j],diff,diff_precent*100))
                    
            else:
                same += 1

    print('total error: %d, same: %d'\
        %(error, same))

    if (len(max_idx)!=0):
        i = max_idx[0]
        j = max_idx[1]
        print("%.6f%%@"%(max*100), end='')
        print("[%d, %d], cpu_value: %f, diff_value: %f"%(i, j, cpu_mat[i,j], abs(cpu_mat[i, j] - gpu_mat[i, j])))

    return


if __name__ == '__main__':

    if len(sys.argv) > 1:
        if sys.argv[1].isdigit():
            print(sys.argv[1])
            compare(int(sys.argv[1]))
        elif sys.argv[1] == 'all':
            for i in range(11):
                compare(i)

    else:
        for i in range(11):
            compare(i)

    