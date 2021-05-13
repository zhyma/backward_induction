import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np


import os
import sys

from sim_tool.py_sim import Vehicle, Load
from sim_tool.search import *

## in the folder quantization_test
## comparing the expectation (value) vs. total cost. Average

if __name__ == "__main__":

    n_d = 256
    n_v = 46
    n_a = 31

    file_name = '128_32_32_cpu'
    # file_name = 'gpu'
    data = Load(file_name)

    N = data.N
    n_d_total = 0
    if n_d == 32:
        n_dc = 37
        n_d_total = 87
    elif n_d == 64:
        n_dc = 75
        n_d_total = 176
    elif n_d == 128:
        n_dc = 149
        n_d_total = 709
    elif n_d == 256:
        n_dc = 299
        n_d_total = 1420
    elif n_d == 512:
        n_dc = 597
        n_d_total = 2843
    line = data.readstate()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))

    mx5=Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2, n_d_total=n_d_total, n_d=n_d, n_v=n_v, n_a=n_a)

    # data.load_value()

    nx = n_d*n_v
    nw = n_dc*2
    
    for i in mx5.a_list:
        print(i)

    print("====")

    for i in mx5.v_list:
        print(i)

    v_max = mx5.v_max
    dt = 2
    dd_list = []
    for v in mx5.v_list:
        for a in mx5.a_list:
            if v+a*dt > v_max:
                v_ = v_max
                t1 = (v_-v)/a
                dd = 0.5*(v+v_)*t1 + v_max*(dt-t1)
            ## speed can't exceed upper bound
            elif v+a*dt < 0:
                v_ = 0
                dd = 0.5*v*(v/(-a))
            ## speed can't be negative
            else:
                v_ = v+a*dt
                dd = 0.5*(v+v_)*dt

            dd_list.append(dd)

    dd_list.sort()
    print(dd_list)