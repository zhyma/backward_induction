import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np
import copy

import os
import sys

## For deterministic example (front call: 80, 110, 140, etc)
## for optimal control, it only run once
## for disturbed policy, it run 10e6 times to get the average


class Vehicle():
    def __init__(self, d2tl, dt, rl_start, rl_end, v_min, v_max, a_min, a_max, N_pred):
        self.d2tl = d2tl
        self.dt = dt
        self.rl_start = rl_start
        self.rl_end = rl_end
        self.t = 0
        self.N = N_pred
        
        self.d = 0
        # boudnary of velocity is [0, v_bound]
        self.v_min = 0.0
        self.v_max = 18.0
        self.v = 0
        self.a_min = -8.0
        self.a_max = 2.0

        self.a_list = self.discretize(a_min, a_max, 32)
        self.v_list = self.discretize(v_min, v_max, 32)
        self.d_list = []
        n_d_total = 353
        for i in range(n_d_total):
            val = i*v_max*10*self.dt/(128-1)
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
        self.v = 0
        return

    def physical(self, a):
        # calculating the next state, but not to move one step forward
        dt = self.dt
        d = self.d
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
            d_ = d + 0.5*(v+v_max)*t1 + v_max*(dt-t1)
        ## speed can't exceed upper bound
        elif v+a*dt < 0:
            v_ = 0
            d_ = d + 0.5*v*(v/(-a))
        ## speed can't be negative
        else:
            v_ = v+a*dt
            d_ = d + 0.5*(v+v_)*dt

        return d_, v_

    def disturb_policy(self, a):
        ak_ = int(np.random.randint(32))
        a_ = self.a_list[ak_]
        # if a_ < 0 and self.v < 1e-3:
        #     _, a_ = self.find_closest(-0.1, self.a_list)
        # if a_ > 0 and self.v >= self.v_max:
        #     _, a_ = self.find_closest(+0.1, self.a_list)

        # _, a_ = self.find_closest(0.5, self.a_list)
        # _, a_ = self.find_closest(0.5, self.a_list)
        return a_

    def running_cost(self, dc, a):
        v0 = self.v
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
        g2 = c*(m*a*v1 + 0.005*m*g*v1 + 0.09*v1**3)*t2
        return g1+g2

    def constraint(self, dc, a):
        # if current state hits the constraint, apply penalty

        d = self.d
        v = self.v
        
        t = self.t
        d2tl = self.d2tl
        t_tcc = 3
        a_min = self.a_min
        a_max = self.a_max

        penalty = False

        # safety distance with the front car
        if d > dc-v*t_tcc:
            penalty = True

        # traffic light condition
        if d < d2tl and t > self.rl_start and t < self.rl_end:
            # check before the red light, if not enough to brake
            if d2tl - d + 0.01 < 0.5*(v**2)/(-a_min):
                penalty = True
            # if in front of a red light, check the acceleration
            u_rlmax = 0.5*v**2/(d2tl-d)
            if a > u_rlmax:
                penalty = True

        return penalty
            

    def terminal_cost(self):
        v = self.v
        d = self.d
        v_max = self.v_max
        d_target = self.N*self.dt*self.v_max

        eta1 = 0.95
        eta2 = 0.95
        t_factor = 783
        term1 = 0.5*self.m*(v_max*v_max-v*v)*eta1
        term2 = (d_target - d)*t_factor
        term3 = self.m*self.g*0*eta2
        return term1+term2+term3
        
    def find_closest(self, val, val_list):
        idx = 0
        find_val = 0
        if val <= val_list[0]:
            idx = 0
            find_val = val_list[0]
        elif val >= val_list[-1]:
            idx = len(val_list)-1
            find_val = val_list[-1]
        else:
            for i in range(len(val_list)-1):
                if val > val_list[i+1]:
                    continue
                else:
                    sub1 = val - val_list[i]
                    sub2 = val_list[i+1] - val
                    if sub1 <= sub2:
                        idx = i
                        find_val = val_list[i]
                        break
                    else:
                        idx = i+1
                        find_val = val_list[i+1]
                        break
        return idx, find_val


    def step_forward(self, a):
        d_, v_ = self.physical(a)
        _, self.d = self.find_closest(d_, self.d_list)
        _, self.v = self.find_closest(v_,self.v_list)
        self.t += self.dt
        return d_, v_

class Load():
    def __init__(self, filepath):
        self.file = filepath
        self.file_handle = open(filepath, 'r')

    def readstate(self):
        return self.file_handle.readline()
    
    def next_trial(self):
        while True :
            line = self.file_handle.readline()
            if (not line) or ('end' in line):
                break
        return

def find_all(name, path):
    result = []
    for root, dirs, files in os.walk(path):
        for f in files:
            if name in f:
                result.append(path + f)
    return result

if __name__ == "__main__":
    # ctrl = Load('output/control.csv')
    action_filename = 'output/cpu_action.csv'
    f_action = open(action_filename, 'r')
    lines = f_action.readlines()
    var = np.fromstring(lines[0], sep=',')[:-1]
    lines = lines[1:]
    N   = int(var[0])
    n_x = int(var[1])
    n_w = int(var[2])

    action_list = []
    for i in range(N):
        action_list.append(lines[i].split(',')[:-1])

    action_mat = np.array(action_list)
    action_mat = action_mat.reshape((N, n_x, n_w))
    print(action_mat.shape)

    curr_dir = os.getcwd() + '/output/'
    files = find_all('front_car_data', curr_dir)
    if len(files) < 1:
        print('No files')
        sys.exit(0)
    traj = Load(files[0])
    
    line = traj.readstate()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))
    gtr_std     = Vehicle(d2tl, 2, rl_start, rl_end, 0, 18, -8, 2, N)
    gtr_disturb = Vehicle(d2tl, 2, rl_start, rl_end, 0, 18, -8, 2, N)
    # count how many trials
    all_cost_disturb = []
    
    front_car_traj = []
    ctrl_cmds = []
    # line = traj.readstate()

    for i in range(N+1):
        traj_list = traj.readstate().split(',')
        pos = int(float(traj_list[0]))
        intention = int(traj_list[-1])
        front_car_traj.append([pos, intention])

    print([i[0] for i in front_car_traj])

    cost2go_std = 0
    for i in range(N):
        dk, _ = gtr_std.find_closest(gtr_std.d, gtr_std.d_list)
        vk, _ = gtr_std.find_closest(gtr_std.v, gtr_std.v_list)
        xk = dk*32+vk
        # read one front car state
        dck, dc = gtr_std.find_closest(front_car_traj[i][0],gtr_std.d_list)
        # dck, _ = gtr_std.find_closest(front_car_traj[i][0], gtr_std.d_list)
        intention = front_car_traj[i][1]
        wk = dck*2+intention
        # find the corresponding ctrl
        ctrl_cmds.append(int(action_mat[i,xk,wk]))
        a = gtr_std.a_list[ctrl_cmds[-1]]
        print('%.2f, '%(a), end='')
        # calculate one running cost
        r_cost = gtr_std.running_cost(dc, a)
        cost2go_std += r_cost
        print("r_cost: %.2f"%(r_cost))
        # if (gtr_std.constraint(front_car_traj[i][0], a)):
        if (gtr_std.constraint(dc, a)):
            cost2go_std += 1e30
            print("Optimal control is not valid, hits the constraint")
        # walk one step
        gtr_std.step_forward(a)

    # print('')
    # terminal cost
    t_cost = gtr_std.terminal_cost()
    print("t_cost: %.2f"%(t_cost))
    cost2go_std += t_cost
    print('cost to go: %.2f (%.3e)'%(cost2go_std, cost2go_std))
    print('%.2f, %.2f'%(gtr_std.d, gtr_std.v))
    print('policy is: ', end='')
    print(ctrl_cmds)
    print('\n')

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

