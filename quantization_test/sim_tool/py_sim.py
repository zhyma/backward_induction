import numpy as np
import sys
import os
from math import ceil

class Vehicle():
    def __init__(self, N_pred, d2tl, dt, rl_start, rl_end, v_min, v_max, a_min, a_max, n_d_total=172, n_d=128, n_v=32, n_a=32):
        self.d2tl = d2tl
        self.dt = dt
        self.rl_start = rl_start
        self.rl_end = rl_end
        self.t = 0
        self.N_pred = N_pred
        
        self.d = 0
        self.xk = 0
        # boudnary of velocity is [0, v_bound]
        self.v_min = v_min
        self.v_max = v_max
        self.v = 0
        self.a_min = a_min
        self.a_max = a_max

        self.a_list = self.discretize(a_min, a_max, n_a)
        self.v_list = self.discretize(v_min, v_max, n_v)
        self.d_list = []
        for i in range(n_d_total):
            val = i*v_max*10*self.dt/(n_d-1)
            self.d_list.append(val)

        self.m = 1500
        self.r = 0.3
        self.g = 9.8
        return

    def discretize(self, min, max, cnt):
        val_list = []
        for i in range(cnt):
            val = min + (max-min)/float(cnt-1)*i
            val_list.append(val)
        return val_list

    def reset(self):
        self.t = 0
        self.d = 0
        self.xk = 0
        self.v = 0
        return


    def physical(self, a, d = -1, v = -1):
        # calculating the next state, but not to move one step forward
        dt = self.dt
        if d < 0:
            d = self.d
        if v < 0:
            v = self.v
        v_max = self.v_max
        if a > self.a_max:
            a = self.a_max
        if a < self.a_min:
            a = self.a_min

        # speed and acceleration is equal or greater than 0
        # or final speed not exceeding upper bound
        if v+a*dt > v_max:
            v_ = v_max
            t1 = (v_-v)/a
            d_ = d + 0.5*(v+v_)*t1 + v_max*(dt-t1)
        ## speed can't exceed upper bound
        elif v+a*dt < 0:
            v_ = 0
            d_ = d + 0.5*v*(v/(-a))
        ## speed can't be negative
        else:
            v_ = v+a*dt
            d_ = d + 0.5*(v+v_)*dt

        return d_, v_

    def running_cost(self, a, v=-1):
        if v < 0:
            v0 = self.v
        else:
            v0 = v
        m = self.m
        g = self.g
        v1 = 0
        t1 = 0
        t2 = 0
        if a > 0:
            c = 1/0.97
            if (self.v_max-v0)/a < self.dt:
                t1 = (self.v_max-v0)/a
                t2 = self.dt - t1
                v1 = self.v_max
            else:
                t1 = self.dt
                t2 = 0
                v1 = v0+a*t1
        elif a == 0:
            # a == 0
            c = 1/0.97
            t1 = 0
            t2 = self.dt-t1
            v1 = v0
        else:
            # a < 0
            c = 0.97 * 0.5
            if v0/(-a) < self.dt:
                t1 = v0/(-a)
                t2 = self.dt - t1
                v1 = 0
            else:
                t1 = self.dt
                t2 = 0
                v1 = v0+a*self.dt

        g1 = 0
        if a != 0:
            g1 = c*(m*a*v0*t1+0.5*m*a*a*t1*t1+0.005*m*g*v0*t1+0.5*0.005*m*g*a*t1*t1+0.09*(v0+a*t1)**4/(4*a))
        g2 = c*(m*a*v1 + 0.005*m*g*v1 + 0.09*(v1**3))*t2
        return g1+g2

    def dist_constraint(self, dc):
        d = self.d
        v = self.v
        
        t_ttc = 3

        penalty = False
        stage1 = 0

        # safety distance with the front car
        if d > dc-v*t_ttc -3:
            penalty = True
            stage1 = dc - d - v*t_ttc -3

        return penalty, stage1

    def rl_constraint(self, k, dc, a):
        # if current state hits the constraint, apply penalty

        d = self.d
        v = self.v
        
        t = self.t
        d2tl = self.d2tl
        t_ttc = 3
        a_min = self.a_min

        penalty = False
        stage1 = 0
        stage2 = 0

        # traffic light condition
        if d < d2tl and t > self.rl_start and t < self.rl_end:
            # check before the red light, if not enough to brake
            stage1 = d2tl - d + 3 - 0.5*(v**2)/(-a_min)
            if d2tl - d + 3 < 0.5*(v**2)/(-a_min):
                penalty = True
            # if in front of a red light, check the acceleration
            if k >= self.N_pred:
                a = a_min
            d_, v_ = self.physical(a)
            stage2 = d2tl - d_ + 3 - 0.5*(v_**2)/(-a_min)
            if d2tl - d_ + 3 < 0.5*(v_**2)/(-a_min):
                penalty = True

        return penalty, stage1, stage2

    def terminal_cost(self, d=-1, v=-1):
        if d < 0:
            d = self.d
        if v < 0:
            v = self.v
        
        v_max = self.v_max
        d_target = self.N_pred*self.dt*self.v_max

        eta1 = 0.95
        eta2 = 0.95
        t_factor = 783
        term1 = 0.5*self.m*(v_max*v_max-v*v)*eta1
        term2 = (d_target - d)*t_factor
        term3 = self.m*self.g*0*eta2
        return term1+term2+term3
        
    def find_closest(self, val, val_list):
        idx = 0
        if val <= val_list[0]:
            idx = 0
        elif val >= val_list[-1]:
            idx = len(val_list)-1
            
        else:
            idx_f = val/(val_list[1]-val_list[0])
            if (float(idx_f) % 1) >= 0.5:
                idx = ceil(idx_f)
            else:
                idx = round(idx_f)

        find_val = val_list[idx]
                
            # for i in range(len(val_list)-1):
            #     if val > val_list[i+1]:
            #         continue
            #     else:
            #         sub1 = val - val_list[i]
            #         sub2 = val_list[i+1] - val
            #         if sub1 <= sub2:
            #             idx = i
            #             find_val = val_list[i]
            #             break
            #         else:
            #             idx = i+1
            #             find_val = val_list[i+1]
            #             break
        return idx, find_val


    def step_forward(self, a):
        d_, v_ = self.physical(a)
        # dk, self.d = self.find_closest(d_, self.d_list)
        # vk, self.v = self.find_closest(v_,self.v_list)
        self.t += self.dt
        self.d = d_
        self.v = v_
        # self.xk = dk*32 + vk
        return d_, v_


def find_all(name, path):
    result = []
    for root, dirs, files in os.walk(path):
        for f in files:
            if name in f:
                result.append(path + f)
    return result

class Load():
    def __init__(self, solver_type):
        self.filename = solver_type
        action_filename = 'output/'+ solver_type +'_action.csv'
        print('load file %s'%(action_filename))
        f_action = open(action_filename, 'r')
        lines = f_action.readlines()
        var = np.fromstring(lines[0], sep=',')[:-1]
        lines = lines[1:]
        self.N   = int(var[0])
        self.n_x_s = int(var[1])
        self.n_w_s = int(var[2])

        action_list = []
        for i in range(self.N):
            action_list.append(lines[i].split(',')[:-1])

        self.action_mat = np.array(action_list)
        self.action_mat = self.action_mat.reshape((self.N, self.n_x_s, self.n_w_s))
        # print(self.action_mat.shape)

        curr_dir = os.getcwd() + '/output/'
        files = find_all('front_car_data', curr_dir)
        if len(files) < 1:
            print('No files')
            sys.exit(0)

        self.file = files[0]
        self.file_handle = open(files[0], 'r')

    def readstate(self):
        return self.file_handle.readline()
    
    def next_trial(self):
        while True :
            line = self.file_handle.readline()
            if (not line) or ('end' in line):
                break
        return

    def load_value(self):
        value_file = 'output/'+ self.filename +'_value.csv'
        print('load file %s'%(value_file))
        f_value = open(value_file, 'r')
        lines = f_value.readlines()
        var = np.fromstring(lines[0], sep=',')[:-1]
        lines = lines[1:]
        self.N   = int(var[0])
        self.n_x = int(var[1])
        self.n_w = int(var[2])

        value_list = []
        # for i in range(self.N):
        #     str_list = lines[i].split(',')[:-1]
        #     value_list.append([float(j) for j in str_list])
        str_list = lines[0].split(',')[:-1]
        value_list.append([float(j) for j in str_list])

        self.value_mat = np.array(value_list)
        # self.value_mat = self.value_mat.reshape((self.N, self.n_x, self.n_w))
        self.value_mat = self.value_mat.reshape((self.n_x, self.n_w))
        # print(self.value_mat.shape)