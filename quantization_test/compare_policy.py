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
    # if len(sys.argv) > 1:
    #     if sys.argv[1] == 'cpu':
    #         solver_type = 'cpu'
    #     elif sys.argv[1] == 'gpu':
    #         solver_type = 'gpu'
    #     else:
    #         solver_type = 'cpu'
    # else:
    #     solver_type = 'cpu'

    gran_type = [32, 64, 128, 256]

    data = []
    mx5 = []
    for i in range(4):
        data.append(Load(str(gran_type[i])))

    N = data[0].N
    line = data[0].readstate()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))

    for i in range(4):
        mx5.append(Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2, gran_type[i]))
    
    # count how many trials
    
    print("\n====\n")

    cnt = 0
    sum = [0] * len(gran_type)
    for k in range(100):
        front_car_traj = []
        for i in range(N+1):
            traj_list = data[0].readstate().split(',')
            pos = int(float(traj_list[0]))
            intention = int(traj_list[-1])
            front_car_traj.append([pos, intention])
        while True:
            state = data[0].readstate()
            if 'end' in state:
                break

        # print([i[0] for i in front_car_traj])
        sto_ctrl = []
        for i in range(4):
            sto_ctrl.append(search_sto(N, data[i].action_mat, mx5[i], front_car_traj))
            total = exam_policy(N, mx5[i], front_car_traj, sto_ctrl[i], verbose = False)
            sum[i] += total

    for i in sum:
        print(i/100)

        # acc_32 = acc_traj(gtr_32, sto_ctrl_a32)


        # fig, ax1 = plt.subplots(figsize=(12,9))
        # title = 'the ' + str(k) + " trajectory: "# + front_car_traj
        # fig.suptitle(title)
        # ax1.plot(list(range(len(acc_32))), acc_32, 'g', label='N_u=32')
        # ax1.plot(list(range(len(acc_64))), acc_64, 'r', label='N_u=64')
        # ax1.plot(list(range(len(acc_128))), acc_128, 'b', label='N_u=128')
        # ax1.plot(list(range(len(acc_256))), acc_256, 'k', label='N_u=256')
        # ax1.legend(loc='upper center', shadow=False, fontsize='x-large')
        # ax1.set_xlabel('k')
        # ax1.set_ylabel('a')
        # ax1.set_ylim([-5, 3])

        # plt.savefig('fig/' + str(k) +'.png')

        # print(total_32)

        # print("\n====\n")
    
    # print("The number of same policy: %d"%(cnt))
