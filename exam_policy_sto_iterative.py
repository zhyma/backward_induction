import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np


import os
import sys

from sim_tool.py_sim import Vehicle, Load
from sim_tool.search import *


## For deterministic example (front call: 80, 110, 140, etc)
## for optimal control, it only run once
## for disturbed policy

## iterate all possible combination of control sequence (for N<=4 maybe?)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == 'cpu':
            solver_type = 'cpu'
        elif sys.argv[1] == 'gpu':
            solver_type = 'gpu'
        else:
            solver_type = 'cpu'
    else:
        solver_type = 'cpu'
    data = Load(solver_type)
    N = data.N
    
    line = data.readstate()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))
    gtr_std     = Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2)
    gtr_disturb = Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2)
    # count how many trials
    
    front_car_traj_1 = []

    for i in range(N+1):
        traj_list = data.readstate().split(',')
        pos = int(float(traj_list[0]))
        intention = int(traj_list[-1])
        front_car_traj_1.append([pos, intention])

    print([i[0] for i in front_car_traj_1])

    sto_policy_1 = search_sto(N, data.action_mat, gtr_std, front_car_traj_1)
    print('----stochastic policy----')
    exam_policy(N, gtr_std, front_car_traj_1, sto_policy_1)
    # best_policy_1 = iterate_action(N, gtr_disturb, front_car_traj_1)
    # print('----best policy----')
    # exam_policy(N, gtr_disturb, front_car_traj_1, best_policy_1)

    print('====case 2====')

    front_car_traj_2 = []

    while True:
        state = data.readstate()
        if 'end' in state:
            break

    for i in range(N+1):
        traj_list = data.readstate().split(',')
        pos = int(float(traj_list[0]))
        intention = int(traj_list[-1])
        front_car_traj_2.append([pos, intention])

    print([i[0] for i in front_car_traj_2])

    sto_policy_2 = search_sto(N, data.action_mat, gtr_std, front_car_traj_2)
    print('----stochastic policy----')
    exam_policy(N, gtr_std, front_car_traj_2, sto_policy_2)
    # best_policy_2 = iterate_action(N, gtr_disturb, front_car_traj_2)
    # print('----best policy----')
    # exam_policy(N, gtr_disturb, front_car_traj_2, best_policy_2)
    

    # print('====crossover====')
    # exam_policy(N, gtr_disturb, front_car_traj_1, best_policy_2)
    # exam_policy(N, gtr_disturb, front_car_traj_2, best_policy_1)