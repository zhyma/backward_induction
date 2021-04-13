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
    # if len(sys.argv) > 1:
    #     if sys.argv[1] == 'cpu':
    #         solver_type = 'cpu'
    #     elif sys.argv[1] == 'gpu':
    #         solver_type = 'gpu'
    #     else:
    #         solver_type = 'cpu'
    # else:
    #     solver_type = 'cpu'
    cpu_data = Load('cpu')
    gpu_data = Load('gpu')
    N = cpu_data.N
    
    line = cpu_data.readstate()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))
    gtr_std     = Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2)
    gtr_disturb = Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2)
    # count how many trials
    
    print("\n====\n")

    cnt = 0
    for k in range(100):
        front_car_traj = []
        for i in range(N+1):
            traj_list = cpu_data.readstate().split(',')
            pos = int(float(traj_list[0]))
            intention = int(traj_list[-1])
            front_car_traj.append([pos, intention])
        while True:
            state = cpu_data.readstate()
            if 'end' in state:
                break

        # print([i[0] for i in front_car_traj])

        sto_policy_cpu = search_sto(N, cpu_data.action_mat, gtr_std, front_car_traj)
        # exam_policy(N, gtr_std, front_car_traj, sto_policy_cpu, verbose = False)

        sto_policy_gpu = search_sto(N, gpu_data.action_mat, gtr_std, front_car_traj)
        if sto_policy_cpu == sto_policy_gpu:
            exam_policy(N, gtr_std, front_car_traj, sto_policy_cpu, verbose = False)
            print("same policy")
            cnt += 1
        else:
            print("different policy")
            print('CPU policy:')
            exam_policy(N, gtr_std, front_car_traj, sto_policy_cpu, verbose = True)
            print('GPU policy:')
            exam_policy(N, gtr_std, front_car_traj, sto_policy_gpu, verbose = True)

        print("\n====\n")
    
    print("The number of same policy: %d"%(cnt))
