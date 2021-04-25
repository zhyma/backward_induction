import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np


import os
import sys

from sim_tool.py_sim import Vehicle, Load
from sim_tool.search import *

def acc_traj(gtr, idx_traj):
	val_traj = []
	for i in idx_traj:
		val_traj.append(gtr.a_list[i])
	return val_traj

## For deterministic example (front call: 80, 110, 140, etc)
## for optimal control, it only run once
## for disturbed policy

## iterate all possible combination of control sequence (for N<=4 maybe?)

if __name__ == "__main__":

    n_d = 128
    n_v = 32
    n_a = 32

    data = Load(str(n_d)+'_'+str(n_v)+'_'+str(n_a)+'_cpu')

    N = data.N
    n_d_total = data.n_w//2
    line = data.readstate()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))

    mx5=Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2, n_d_total=n_d_total, n_d=n_d, n_v=n_v, n_a=n_a)
    
    # count how many trials
    
    print("\n====\n")

    cnt = 0
    sum = 0
    for k in range(100):
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

        # print([i[0] for i in front_car_traj])
        sto_ctrl = []

        sto_ctrl=search_sto(N, data.action_mat, mx5, front_car_traj)
        total = exam_policy(N, mx5, front_car_traj, sto_ctrl, verbose = False)
        if total < 1e14:
            sum += total
            cnt += 1

    print(cnt)
    print(sum/cnt)


