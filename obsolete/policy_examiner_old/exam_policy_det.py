import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np
import copy

import os
import sys

from sim_tool.py_sim import Vehicle, Load
from sim_tool.search import *

## For deterministic example (front call: 80, 110, 140, etc)
## for optimal control, it only run once
## for disturbed policy, it run 10e6 times to get the average

def get_v0(xk, wk):
    f = open('output/cpu_value.csv','r') 
    lines = f.readlines()

    dim = np.fromstring(lines[0], sep=',')
    N   = int(dim[0])
    n_x = int(dim[1])
    n_w = int(dim[2])

    # only need the data at k = 0
    mat = np.fromstring(lines[1], sep=',')[:-1]

    mat = mat.reshape(n_x,n_w)

    return mat[xk,wk]

if __name__ == "__main__":
    data = Load()
    N = data.N
    
    line = data.readstate()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))
    gtr_std     = Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2)
    gtr_disturb = Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2)
    
    front_car_traj = []
    ctrl_cmds = []
    # line = traj.readstate()

    for i in range(N+1):
        traj_list = data.readstate().split(',')
        pos = int(float(traj_list[0]))
        intention = int(traj_list[-1])
        front_car_traj.append([pos, intention])

    print([i[0] for i in front_car_traj])

    sto_policy = search_sto(N, data.action_mat,gtr_std, front_car_traj)
    print('----stochastic policy----')
    exam_policy(N, gtr_std, front_car_traj, sto_policy)

    ## start to simulate with disturbed policy

    min_disturb_cost =1e20
    best_disturb_policy = []

    cnt = 0
    valid_cnt = 0
    invalid_cnt = 0

    iter = 1000*1000*10
    for test in range(iter):
        # each trial contain 10 control steps
        cost2go_disturb = 0
        disturb_policy_list = []
        gtr_disturb.reset()
        valid_ctrl = True

        # test_a = [31,31,29,28]
        for i in range(N): 
            dck, _ = gtr_std.find_closest(front_car_traj[i][0], gtr_disturb.d_list)
            dc =  gtr_disturb.d_list[dck]
            a = gtr_disturb.a_list[ctrl_cmds[i]]

            a_disturb = gtr_disturb.disturb_policy(a)
            disturb_policy_list.append(a_disturb)
            # print('disturbed: d is %.2f, v is %.2f, a is: %.2f'%(gtr_disturb.d, gtr_disturb.v, a_disturb))
            r_cost = gtr_disturb.running_cost(dc, a_disturb)
            # print("action:%.2f, \nr_cost: %.2f"%(a_disturb, r_cost))
            cost2go_disturb += r_cost
            _, front_car_state_val = gtr_disturb.find_closest(front_car_traj[i][0], gtr_disturb.d_list)
            if (gtr_disturb.constraint(front_car_state_val, a)):
                # print("hit the constraint %.2f, %.2f, with a=%.2f"%(gtr_disturb.d, gtr_disturb.v,a_disturb))
                # print("front car position %.2f"%(front_car_traj[i][0]))
                cost2go_disturb += 1e30
                # valid_ctrl = False
                # break
            gtr_disturb.step_forward(a_disturb)
        
        # print('; ', end='')

        if valid_ctrl == True:
            valid_cnt += 1
            # print('disturbed: d is %.2f, v is %.2f, a is: %.2f'%(gtr_disturb.d, gtr_disturb.v, a_disturb))
            t_cost = gtr_disturb.terminal_cost()
            cost2go_disturb += t_cost
            # print("t_cost: %.2f"%(t_cost))
            # print("cost to go is %.2f\n"%(cost2go_disturb))
            if cost2go_disturb < min_disturb_cost:
                min_disturb_cost = cost2go_disturb
                best_disturb_policy = copy.deepcopy(disturb_policy_list)

            all_cost_disturb.append(cost2go_disturb)

            if (cost2go_std > cost2go_disturb):
                cnt += 1
                break
        else:
            invalid_cnt += 1

        if test%(iter/10)==0:
            print(f"{test/(iter/10):.0f}0%...")
        

    print("There are %d cost2go is greater than optimal"%(cnt))
    print("Find out %d valid control and %d invalid control"%(valid_cnt, invalid_cnt))

    print("distubed mean: %.3e, min: %.2f"%(np.mean(all_cost_disturb), min(all_cost_disturb)))
    print("best disturbed policy is: ")
    print(best_disturb_policy)
    for i in best_disturb_policy:
        a, _ = gtr_disturb.find_closest(i, gtr_disturb.a_list)
        print(a,end=', ')
    print('\n')

