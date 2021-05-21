import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np


import os
import sys

from sim_tool.py_sim import Vehicle, Load, find_all
from sim_tool.search import *
from rule_based_ctrl import a_ctrl

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

    trials = 100

    data = []
    mx5 = []
    # for i in range(4):
    #     data.append(Load('128_32_'+str(gran_type[i])))

    # N = data[0].N
    # n_d_total = data[0].n_w//2
    # print(n_d_total)
    # line = data[0].readstate()
    # param = line.split(',')
    # d2tl = float(param[0].split('=')[1])
    # rl_start = float(param[1].split('=')[1])
    # rl_end = float(param[2].split('=')[1])
    # print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))

    curr_dir = os.getcwd() + '/output/'
    files = find_all('front_car_data', curr_dir)
    if len(files) < 1:
        print('No files')
        sys.exit(0)

    file_handle = open(files[0], 'r')

    N = 10
    line = file_handle.readline()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    n_d_total = 1001
    n_d = 361
    n_v = 46
    n_a = 31

    print(d2tl)

    mx5 = Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2, n_d_total=n_d_total, n_d=n_d, n_v=n_v, n_a=n_a)

    ctrller = a_ctrl(mx5)
    
    front_car_traj = []
    # for i in range(N+1):
    #     traj_list = file_handle.readline().split(',')
    #     pos = float(traj_list[0])
    #     intention = int(traj_list[-1])
    #     front_car_traj.append([pos, intention])
    # while True:
    #     state = data.readstate()
    #     if 'end' in state:
    #         break

    # dc0 = front_car_traj[0][0]
    # i0 = front_car_traj[0][1]
    # dck0, dc0_ = mx5.find_closest(dc0, mx5.d_list)
    # wk = dck0*2+i0
    # # print('dc0 = %d, intention = %d, dck0 = %d, dck0, dc0=%f'%(dc0, i0, dck0, dc0_))

    # do rest of the trajectory
    cost_sum = 0
    cost_cnt = 0
    for k in range(trials-1):
        front_car_traj = []
        for i in range(N+1):
            traj_list = file_handle.readline().split(',')
            pos = float(traj_list[0])
            v = float(traj_list[1])
            intention = int(traj_list[-1])
            front_car_traj.append([pos, intention, v])
        while True:
            state = file_handle.readline()
            if 'end' in state:
                break

        # print([i[0] for i in front_car_traj])

        # Getting the total cost for the physical system
        ctrller.reset()
        ctrl_seq = []
        j = 0
        for i in front_car_traj:
            # print(j)
            # j+=1
            # print("front car now: %f"%(dc))

            if ctrller.est_dc_ > i[0] or ctrller.est_vc_ > i[2]:
                print("front car: d=%.2f, v=%.2f"%(i[0], i[2]))
                print("estim car: d=%.2f, v=%.2f"%(ctrller.est_dc_, ctrller.est_vc_))
                print("estimate error!!!")
                print("")
            
            ak = ctrller.gen_ctrl(i[0], i[1])
            # if ctrller.est_dc > i[0] or ctrller.est_vc > i[2]:
            #     print("front car: d=%.2f, v=%.2f"%(i[0], i[2]))
            #     print("estim car: d=%.2f, v=%.2f"%(ctrller.est_vc_))
            #     print("estimate error!!!")
            #     print("")
            # if ctrller.est_dc > i[0] or ctrller.est_vc > i[2]:

            a = mx5.a_list[ak]
            mx5.step_forward(a)
            # print("ego car: %f, %f\n"%(mx5.d, mx5.v))
            ctrl_seq.append(ak)


        # print(ctrl_seq)
        
        # print([x for x in sto_ctrl])
        total = exam_policy(N, mx5, front_car_traj, ctrl_seq, loose = True, verbose = False)
        # print(total)
        if total < 1e14:
            cost_sum += total
            cost_cnt += 1

        # print(total, end='\n')

    print(cost_sum/cost_cnt)
