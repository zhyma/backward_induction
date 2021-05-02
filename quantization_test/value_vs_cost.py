import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np


import os
import sys

from sim_tool.py_sim import Vehicle, Load
from sim_tool.search import *

## comparing the expectation (value) vs. total cost. Average

if __name__ == "__main__":

    n_d = 512
    n_v = 32
    n_a = 32

    file_name = str(n_d)+'_'+str(n_v)+'_'+str(n_a)+'_cpu'
    # file_name = 'gpu'
    data = Load(file_name)

    N = data.N
    n_d_total = 0
    if n_d == 32:
        n_d_total = 87
    elif n_d == 64:
        n_d_total = 176
    elif n_d == 128:
        n_d_total = 709
    elif n_d == 256:
        n_d_total = 1420
    elif n_d == 512:
        n_d_total = 2843
    line = data.readstate()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))

    mx5=Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2, n_d_total=n_d_total, n_d=n_d, n_v=n_v, n_a=n_a)
    
    # count how many trials
    
    print("\n====\n")

    # Getting the average total cost for the physical system
    cost_cnt = 0
    cost_sum = 0

    value_sum = 0

    # data.load_value()

    for k in range(1):
        front_car_traj = []
        for i in range(N+1):
            traj_list = data.readstate().split(',')
            pos = int(float(traj_list[0]))
            intention = int(traj_list[-1])
            front_car_traj.append([pos, intention])
        while True:
            state = data.readstate()
            if 'end' in state:
                break

        print([i[0] for i in front_car_traj])

        # Getting the total cost for the physical system
        sto_ctrl = []

        sto_ctrl=search_sto(N, data.action_mat, mx5, front_car_traj)
        total = exam_policy(N, mx5, front_car_traj, sto_ctrl, loose = True, verbose = True)
        if total < 1e14:
            cost_sum += total
            cost_cnt += 1

        # exam_value(N, mx5, front_car_traj, sto_ctrl, data.value_mat)

        dc0 = front_car_traj[0][0]
        i0 = front_car_traj[0][1]
        dck0, dc0_ = mx5.find_closest(dc0, mx5.d_list)
        wk = dck0*2+i0
        # print('dc0 = %d, intention = %d, dck0 = %d, dck0, dc0=%f'%(dc0, i0, dck0, dc0_))
        value_sum += data.value_mat[0,0,wk]
        # print(total)
        # print(data.value_mat[0,0,wk])
        # print('----')

    print(cost_cnt)
    print(cost_sum/cost_cnt)
    print('====')
    print(value_sum)

    



