import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np
import copy


## For deterministic example (front call: 80, 110, 140, etc)
## for optimal control, it only run once
## for disturbed policy, it run 10e6 times to get the average


class Vehicle():
    def __init__(self, d2tl, dt, rl_start, rl_end, v_min, v_max, a_min, a_max):
        self.d2tl = d2tl
        self.dt = dt
        self.rl_start = rl_start
        self.rl_end = rl_end
        self.t = 0
        
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
        n_d = 353
        for i in range(n_d):
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
        return a_

    def running_cost(self, dc, a):
        v0 = self.v
        m = self.m
        g = self.g
        if a > 0:
            c = 1/0.97/4.71 * (5*np.pi)
            if (self.v_max-v0)/a > self.dt:
                t1 = (self.v_max-v0)/a
                t2 = self.dt - t1
            else:
                t1 = self.dt
                t2 = 0
        elif a < 0:
            # a < 0
            c = 0.97/4.71 * 0.5 * (5*np.pi)
            t1 = v0/(-a)
            t2 = self.dt - t1
        else:
            # a == 0
            c = 1/0.97/4.71 * (5*np.pi)
            t1 = 0
            t2 = self.dt

        v1 = v0+a*t1

        if a == 0:
            g1 = 0
        else:
            g1 = c*(m*a*v0*t1+0.5*m*a**2*t1**2+0.005*m*g*v0*t1+0.5*0.005*m*g*a*t1**2+0.09*(v0+a*t1)**4/(4*a))
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
        if dc-self.v*t_tcc < d:
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
        d_target = 10*self.dt*self.v_max

        eta1 = 0.95
        eta2 = 0.95
        t_factor = 783
        term1 = 1/2*self.m*(self.v_max**2-v**2)*eta1
        term2 = (d_target - d)*t_factor
        term3 = self.m*self.g*0*eta2
        return term1+term2+term3
        
    def find_closest(self, val, val_list):
        if val <= val_list[0]:
            return val_list[0]
        elif val >= val_list[-1]:
            return val_list[-1]
        else:
            for i in range(len(val_list)-1):
                if val > val_list[i+1]:
                    continue
                else:
                    sub1 = val - val_list[i]
                    sub2 = val_list[i+1] - val
                    if sub1 <= sub2:
                        return val_list[i]
                    else:
                        return val_list[i+1]


    def step_forward(self, a):
        d_, v_ = self.physical(a)
        self.d = self.find_closest(d_, self.d_list)
        self.v = self.find_closest(v_,self.v_list)
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

if __name__ == "__main__":
    ctrl = Load('output/control.csv')
    traj = Load('output/front_car_data.csv')
    
    line = traj.readstate()
    param = line.split(',')
    d2tl = float(param[0].split('=')[1])
    rl_start = float(param[1].split('=')[1])
    rl_end = float(param[2].split('=')[1])
    print("%.1f, %.1f, %.1f"%(d2tl, rl_start, rl_end))
    gtr_std = Vehicle(d2tl, 2, rl_start, rl_end, 0, 18, -8, 2)
    gtr_disturb = Vehicle(d2tl, 2, rl_start, rl_end, 0, 18, -8, 2)
    # count how many trials
    trial_cnt = 0
    all_cost_std = []
    all_cost_disturb = []
    
    front_car_traj = []
    ctrl_cmds = []
    # line = traj.readstate()

    for i in range(10):
        front_car_traj.append(float(traj.readstate().split(',')[0]))
        ctrl_cmds.append(float(ctrl.readstate().split('\n')[0]))

    print(front_car_traj)
    print(ctrl_cmds)

    min_disturb_cost =1e5
    best_disturb_policy = []

    cnt = 0
    cost2go_std = 0
    for i in range(10):
        dc = front_car_traj[i]
        a  = ctrl_cmds[i]
        cost2go_std += gtr_std.running_cost(dc, a)
        if (gtr_std.constraint(dc, a)):
            cost2go_std += 1e30
            print("Optimal control is not valid, hits the constraint")
        # print('optimal: d is %.2f, v is %.2f, a is: %.2f'%(gtr_std.d, gtr_std.v, a))
        gtr_std.step_forward(a)


    for test in range(1000*1000):
        # each trial contain 10 control steps
        cost2go_disturb = 0

        disturb_policy_list = []
        
        for i in range(10): 
            dc = front_car_traj[i]
            a  = ctrl_cmds[i]

            a_disturb = gtr_disturb.disturb_policy(a)
            disturb_policy_list.append(a_disturb)
            # print('disturbed: d is %.2f, v is %.2f, a is: %.2f'%(gtr_disturb.d, gtr_disturb.v, a_disturb))
            cost2go_disturb += gtr_disturb.running_cost(dc, a_disturb)
            if (gtr_disturb.constraint(dc, a)):
                cost2go_disturb += 1e30
            gtr_disturb.step_forward(a_disturb)


        if cost2go_disturb < min_disturb_cost:
            min_disturb_cost = cost2go_disturb
            best_disturb_policy = copy.deepcopy(disturb_policy_list)

        all_cost_disturb.append(cost2go_disturb)
        gtr_disturb.reset()

        if (cost2go_std > cost2go_disturb):
            cnt += 1

    print(cnt)
    # fig, (ax1, ax2) = plt.subplots(1, 2, gridspec_kw={'width_ratios': [1, 1]}, figsize=(12,9))

    # ax1.hist(all_cost_std, 1000, alpha=0.5, label='std')
    print("cost2go: %.3e"%(cost2go_std))
    print("distubed mean: %.3e, min: %.3e"%(np.mean(all_cost_disturb), min(all_cost_disturb)))
    print("best disturbed policy is: ")
    print(best_disturb_policy)
    # plt.hist(all_cost_disturb, 10, alpha=0.5, label='disturbed')
    # plt.legend(loc='upper right')
    # plt.show()
    

