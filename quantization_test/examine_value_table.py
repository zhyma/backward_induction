import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np


import os
import sys

from sim_tool.py_sim import Vehicle, Load
from sim_tool.search import *

## comparing the expectation (value) vs. total cost. Average

if __name__ == "__main__":

    n_d = 241
    n_v = 46
    n_a = 31

    # file_name = str(n_d)+'_'+str(n_v)+'_'+str(n_a)+'_cpu'
    file_name = '241_'+str(n_v)+'_'+str(n_a)+'_cpu'
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
    elif n_d == 31:
        n_dc = 36
        n_d_total = 84
    elif n_d == 61:
        n_dc = 71
        n_d_total = 168
    elif n_d == 32:
        n_dc = 37
        n_d_total = 87
    elif n_d == 64:
        n_dc = 75
        n_d_total = 176
    elif n_d == 128:
        n_dc = 149
        n_d_total = 709
    elif n_d == 256:
        n_dc = 299
        n_d_total = 1420
    elif n_d == 512:
        n_dc = 597
        n_d_total = 2843
    line = data.readstate()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))

    mx5=Vehicle(N, d2tl, 2, rl_start, rl_end, 0, 18, -4, 2, n_d_total=n_d_total, n_d=n_d, n_v=n_v, n_a=n_a)

    # data.load_value()

    nx = n_d*n_v
    nw = n_dc*2
    
    # for k in range(N+1):
    #     for i in range(nx):
    #         for j in range(nw):
    #             if abs(data.value_mat[k, i, j]-91513) < 2:
    #                 d = mx5.d_list[i//n_v]
    #                 v = mx5.v_list[i%n_v]
    #                 dc = mx5.d_list[j//2]
    #                 intention = j%2
    #                 print('find %d at k=%d, xk=%d, dk=%d, vk=%d, dck=%d, intention=%d'%(data.value_mat[k,i,j], k, i, i//n_v, i%n_v, j//2, intention))
    #                 print('find %d at k=%d, d=%f, v=%f, dc=%f, intention=%d'%(data.value_mat[k,i,j], k, d, v, dc, intention))

    #             # if abs(data.value_mat[k, i, j]-64878) < 2:
    #             #     d = mx5.d_list[i//n_v]
    #             #     v = mx5.v_list[i%n_v]
    #             #     dc = mx5.d_list[j//2]
    #             #     intention = j%2
    #             #     print('find %d at k=%d, d=%f, v=%f, dc=%f, intention=%d'%(data.value_mat[k,i,j], k, d, v, dc, intention))
    
    
    # dk = 85
    # vk = 40
    # ak = 25
    # dx = mx5.d_list[dk]
    # vx = mx5.v_list[vk]
    # ax = mx5.a_list[ak]
    # d_, v_ = mx5.physical(ax, d=dx, v=vx)
    # print("d=%f, v=%f, a=%f"%(dx, vx, ax))
    # # print("d_=%f, v_=%f"%(d_, v_))
    # dk_, d_2 = mx5.find_closest(d_, mx5.d_list)
    # vk_, v_2 = mx5.find_closest(v_, mx5.v_list)
    # print("dk_=%d, vk_=%d"%(dk_, vk_))
    # print("d_2=%f, v_2=%f"%(d_2, v_2))
    # print("xk=%d"%(dk_*n_v+vk_))
    # r = mx5.running_cost(ax, v=vx)
    # print(r)

    # dk = 78
    # vk = 25
    # dx = mx5.d_list[dk]
    # vx = mx5.v_list[vk]
    # t = mx5.terminal_cost(d=dx, v=vx)
    # print(t)

    # idx, val = mx5.find_closest(197.31, mx5.d_list)
    # print("%d, %f"%(idx, val))

    # dk, d = mx5.find_closest(350, mx5.d_list)
    # print("dk=%d, d=%f"%(dk, d))
    # vk, v = mx5.find_closest(8, mx5.v_list)
    # print("vk=%d, v=%f"%(vk, v))
    # ak, a = mx5.find_closest(2, mx5.a_list)
    # print("ak=%d, a=%f"%(ak, a))
    # # t = mx5.terminal_cost(d=d,v=v)
    # # print(t)
    # r = mx5.running_cost(a, v=v)
    # print(r)

    for k in range(1):
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

        print([i[0] for i in front_car_traj])

    #     # Getting the total cost for the physical system
    #     sto_ctrl = []

        # sto_ctrl=search_sto(N, data.action_mat, mx5, front_car_traj)
        # print(sto_ctrl)
        # new_sto = [31,31,31,29,28,20,20,20,20,20]
        new_sto = [30,27,20,20,0,30,30,22,20,0]
        total = exam_policy(N, mx5, front_car_traj, new_sto, loose = True, verbose = True)
        print(total)