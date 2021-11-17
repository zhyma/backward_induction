import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np

import os
import sys

from sim_tool.py_sim import Vehicle, Load
from sim_tool.search import *
from rule_based_ctrl import *

## visualizing the front car, policy-based, rule-based trajectories.

if __name__ == "__main__":

    # trials = int(sys.argv[1])

    trials = 100
    n_d = 361
    n_v = 46
    n_a = 31

    file_name = str(n_d)+'_'+str(n_v)+'_'+str(n_a)+'_gpu'
    # file_name = 'gpu'
    data = Load(file_name)

    N = data.N
    n_d_total = 0
    if n_d == 121:
        n_dc = 141
        n_d_total = 334
    elif n_d == 241:
        n_dc = 281
        n_d_total = 668
    elif n_d == 361:
        n_dc = 421
        n_d_total = 1001

    line = data.readstate()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    # print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))

    # mx5 using sdp solved policy
    mx5=Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2, n_d_total=n_d_total, n_d=n_d, n_v=n_v, n_a=n_a)
    
    # print("\n====\n")

    # Getting the average total cost for the physical system
    bi_cost_cnt = 0
    bi_cost_sum = 0

    rule_cost_cnt = 0
    rule_cost_sum = 0

    value = 0

    # data.load_value()

    front_car_traj = []

    t = list(range(0, 21, 2))
    # do rest of the trajectory
    for k in range(trials):
        front_car_traj = []
        for i in range(N+1):
            traj_list = data.readstate().split(',')
            pos = float(traj_list[0])
            intention = int(traj_list[-1])
            front_car_traj.append([pos, intention])
        while True:
            state = data.readstate()
            if 'end' in state:
                break

        # print([i[0] for i in front_car_traj])

        # Getting the total cost for the physical system
        bi_ctrl = []

        bi_ctrl = search_sto(N, data.action_mat, mx5, front_car_traj)
        # print(bi_ctrl)
        # print([x for x in sto_ctrl])
        total, bi_traj = exam_policy(N, mx5, front_car_traj, bi_ctrl, loose = True, verbose = False)

        # print(front_car_traj)

        rule_ctrl = ctrl_seq(front_car_traj, mx5)
        # print(rule_ctrl)
        total, rule_traj = exam_policy(N, mx5, front_car_traj, rule_ctrl, loose = True, verbose = False)

        if (k==0 or k==3 or k==4 or k==12):
            fig, ax1 = plt.subplots()
            ax1.plot(t, [i[0] for i in front_car_traj], color = 'crimson', marker='o', label='leading vehicle trajectory')
            print([i[0] for i in front_car_traj])
            ax1.plot(t, bi_traj, color = 'dodgerblue', marker='^', label='policy-based trajectory')
            print(bi_traj)
            ax1.plot(t, rule_traj, color = 'orange', marker='s', label='rule-based trajectory')
            print(rule_traj)
            ax1.plot([0,20],[150,150], 'k--')
            ax1.plot([10,10],[0,360], 'k--')
            ax1.set_xlabel('t(s)')
            ax1.set_ylabel('distance(m)')
            ax1.set_ylim([0,360])
            fig.tight_layout()
            ax1.legend(loc='upper left')
            print('----')
            # plt.show()

            # plt.savefig('fig/' + 'traj_'+str(k)+'.png')
        
