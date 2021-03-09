import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np

def disturb_policy(a):
    if a < -1:
        a = a+1
    elif a > 1:
        a = a-1

    return a


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

        self.m = 1500
        self.r = 0.3
        self.g = 9.8
        return

    def running_cost(self, a, dc):
        v0 = self.v
        m = self.m
        g = self.g
        if a >= 0:
            c = 1/0.97/4.71 * (5*np.pi*self.r)
            if (self.v_max-v0)/a > self.dt:
                t1 = (self.v_max-v0)/a
                t2 = self.dt - t1
            else:
                t1 = self.dt
                t2 = 0
        else:
            # a < 0
            t1 = v0/(-a)
            t2 = self.dt - t1
            c = 0.97/4.71 * 0.5 * (5*np.pi*self.r)

        v1 = v0+a*t1

        g1 = c*(m*a*v0*t1+0.5*m*a**2*t1**2+0.005*m*g*v0*t1+0.5*0.005*m*g*a*t1**2+0.09*(v0+a*t1)**4/(4*a))
        g2 = c*(m*a*v1 + 0.005*m*g*v1 + 0.09*v1**3)*t2
        return g1+g2

    def constraint(self, dc):
        pass

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
        

    def sim_step(self, a):
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

        self.d = d_
        self.v = v_
        return d_, v_

class Load():
    def __init__(self, filepath):
        self.file_handle = open(filepath, 'r')

    def readstate(self):
        return self.file_handle.readline()
    
    def next_trial(self):
        while True :
            line = self.file_handle.readline()
            if (not line) or (line == 'end'):
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
    while True:
        # skip the front car information at t=0
        line = traj.readstate()
        if not line:
            # end of file, exit
            break

        trial_cnt += 1
        # each trial contain 10 control steps
        cost2go_std = 0
        cost2go_disturb = 0
        for i in range(10):
            dc = float(traj.readstate().split(',')[0])
            a = float(ctrl.readstate())
            a_disturb = disturb_policy(a)
            cost2go_std += gtr_std.running_cost(a, dc)
            cost2go_disturb += gtr_disturb.running_cost(a_disturb, dc)

        all_cost_std.append(cost2go_std)
        all_cost_disturb.append(cost2go_disturb)

        ctrl.next_trial()
        traj.next_trial()

    sum_std = sum(all_cost_std)
    sum_disturb = sum(all_cost_disturb)
    print('Test %d trials'%(trial_cnt))
    print('"optimal control" gets %f'%(sum_std))
    print('"optimal control" gets %f'%(sum_std))
    if sum_std > sum_disturb:
        print('sum_std > sum_disturb')
    elif sum_std == sum_disturb:
        print('sum_std == sum_disturb')
    else:
        print('sum_std < sum_disturb')


