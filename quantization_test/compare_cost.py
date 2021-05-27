import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import sem

import os
import sys

from sim_tool.py_sim import Vehicle, Load
from sim_tool.search import *

## input: two policy, compare the expectation

if __name__ == "__main__":

    file1 = '128_32_32_cpu'
    file2 = '512_32_32_cpu'
    data1 = Load(file1)
    data2 = Load(file2)

    N = data1.N
    n_d_total1 = data1.n_w//2
    n_d_total2 = data2.n_w//2
    line = data1.readstate()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))

    mx128 = Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2, n_d_total=n_d_total1, n_d=128, n_v=32, n_a=32)
    mx512 = Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2, n_d_total=n_d_total2, n_d=512, n_v=32, n_a=32)
    
    # count how many trials
    
    print("\n====\n")

    cnt1 = 0
    cnt2 = 0
    cnt3 = 0
    diff = []
    for k in range(100):
        front_car_traj = []
        for i in range(N+1):
            traj_list = data1.readstate().split(',')
            pos = int(float(traj_list[0]))
            intention = int(traj_list[-1])
            front_car_traj.append([pos, intention])
        while True:
            state = data1.readstate()
            if 'end' in state:
                break

        # print([i[0] for i in front_car_traj])
        sto_ctrl1 = []
        sto_ctrl2 = []

        

        sto_ctrl1=search_sto(N, data1.action_mat, mx128, front_car_traj)
        sto_ctrl2=search_sto(N, data2.action_mat, mx512, front_car_traj)
        total1, _ = exam_policy(N, mx128, front_car_traj, sto_ctrl1, loose=True, verbose = False)
        total2, _ = exam_policy(N, mx512, front_car_traj, sto_ctrl2, loose=True, verbose = False)
        if total1 > 1e14 and total2 > 1e14:
            print('all hits the constraint')
        elif total1 < 1e14 and total2 > 1e14:
            print('constraint error!')
        elif total1 > 1e14 and total2 < 1e14:
            print('constraint error!')
        else:
            diff.append(abs(total1-total2))
            if abs(total1-total2) < 1:
                cnt3 += 0
            elif total1 > total2:
                cnt1 += 1
            elif total1 < total2:
                cnt2 += 1
            
        

    print(cnt1)
    print(cnt2)
    print(cnt3)
    print('====')
    print(np.mean(diff))
    print(sem(diff))


