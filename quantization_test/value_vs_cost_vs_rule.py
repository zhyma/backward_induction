import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np

import os
import sys

from sim_tool.py_sim import Vehicle, Load
from sim_tool.search import *
from rule_based_ctrl import *

## comparing the expectation (value) vs. total cost. Average

if __name__ == "__main__":

    trials = int(sys.argv[1])
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
    elif n_d == 481:
        n_dc = 561
        n_d_total = 1334
    elif n_d == 601:
        n_dc = 701
        n_d_total = 1668
    elif n_d == 721:
        n_dc = 841
        n_d_total = 2001

    line = data.readstate()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    # print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))

    # mx5 using sdp solved policy
    mx5=Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2, n_d_total=n_d_total, n_d=n_d, n_v=n_v, n_a=n_a)
    # rx7 using 
    rx7=Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2, n_d_total=n_d_total, n_d=n_d, n_v=n_v, n_a=n_a)
    # count how many trials
    
    # print("\n====\n")

    # Getting the average total cost for the physical system
    bi_cost_cnt = 0
    bi_cost_sum = 0

    rule_cost_cnt = 0
    rule_cost_sum = 0

    value = 0

    data.load_value()

    # get the first trajectory, and the value
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

    dc0 = front_car_traj[0][0]
    i0 = front_car_traj[0][1]
    dck0, dc0_ = mx5.find_closest(dc0, mx5.d_list)
    wk = dck0*2+i0
    # print('dc0 = %d, intention = %d, dck0 = %d, dck0, dc0=%f'%(dc0, i0, dck0, dc0_))
    value = data.value_mat[0,wk]

    # do rest of the trajectory
    for k in range(trials-1):
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
        # print([x for x in sto_ctrl])
        total = exam_policy(N, mx5, front_car_traj, bi_ctrl, loose = True, verbose = False)
        # print(total)
        if total < 1e14:
            bi_cost_sum += total
            bi_cost_cnt += 1

        rule_ctrl = ctrl_seq(front_car_traj, mx5)
        total = exam_policy(N, mx5, front_car_traj, rule_ctrl, loose = True, verbose = False)
        if total < 1e14:
            rule_cost_sum += total
            rule_cost_cnt += 1

    print(bi_cost_cnt)
    bi_cost_avg = bi_cost_sum/bi_cost_cnt
    print('average bi_solver cost:            %.2f'%(bi_cost_avg))
    print('corresponding value:     %.2f'%(value))
    diff = (bi_cost_avg-value)/bi_cost_avg
    print('estimated difference is: %.2f%%'%(diff*100))

    print(rule_cost_cnt)
    rule_cost_avg = rule_cost_sum/rule_cost_cnt
    print('average rule based cost:            %.2f'%(rule_cost_avg))
    diff = (rule_cost_avg-bi_cost_avg)/bi_cost_avg
    print('estimated difference is: %.2f%%'%(diff*100))

    



