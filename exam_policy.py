import numpy as numpy

import matplotlib.pyplot as plt
import numpy as np

def disturb_policy(car, a):
    a_ = a + np.random.uniform(-4, 2, 1)
    if a_ > 2:
        a_ = 2
    elif a_ < -4:
        a_ = -4

    if car.v <= car.v_min and a_ < 0:
        a_ = 0
    elif car.v >= car.v_max and a_ > 0:
        a_ = 0
    return a_


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
        self.a_min = -4.0
        self.a_max = 2.0

        self.m = 1500
        self.r = 0.3
        self.g = 9.8
        return

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

    def running_cost(self, dc, a):
        v0 = self.v
        m = self.m
        g = self.g
        if a > 0:
            c = 1/0.97/4.71 * (5*np.pi*self.r)
            if (self.v_max-v0)/a > self.dt:
                t1 = (self.v_max-v0)/a
                t2 = self.dt - t1
            else:
                t1 = self.dt
                t2 = 0
        elif a < 0:
            # a < 0
            c = 0.97/4.71 * 0.5 * (5*np.pi*self.r)
            t1 = v0/(-a)
            t2 = self.dt - t1
        else:
            # a == 0
            c = 1/0.97/4.71 * (5*np.pi*self.r)
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
        

    def step_forward(self, a):
        d_, v_ = self.physical(a)
        self.d = d_
        self.v = v_
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
    gtr_std = Vehicle(d2tl, 2, rl_start, rl_end, 0, 18, -4, 2)
    gtr_disturb = Vehicle(d2tl, 2, rl_start, rl_end, 0, 18, -4, 2)
    # count how many trials
    trial_cnt = 0
    all_cost_std = []
    all_cost_disturb = []
    
    end_flag = False
    while end_flag == False:
        # skip the front car information at t=0
        # line = traj.readstate()
        if not line:
            # end of file, exit
            end_flag = True
            break
        
        # each trial contain 10 control steps
        cost2go_std = 0
        cost2go_disturb = 0
        
        for i in range(10):
            traj_state = traj.readstate().split(',')[0]
            ctrl_state = ctrl.readstate()
            if traj_state == '' or ctrl_state == '':
                end_flag = True
                break
            elif i == 0:
                trial_cnt += 1
                print("Solving iteration %d"%(trial_cnt))
            dc = float(traj_state)
            a  = float(ctrl_state)
            cost2go_std += gtr_std.running_cost(dc, a)
            if (gtr_std.constraint(dc, a)):
                cost2go_std += 1e30
                print("Optimal control is not valid, hits the constraint")
            gtr_std.step_forward(a)

            a_disturb = disturb_policy(gtr_disturb, a)
            print('d is %.2f, v is %.2f, disturbed a is: %.2f'%(gtr_disturb.d, gtr_disturb.v, a_disturb))
            cost2go_disturb += gtr_disturb.running_cost(dc, a_disturb)
            if (gtr_disturb.constraint(dc, a)):
                cost2go_disturb += 1e30
            gtr_disturb.step_forward(a_disturb)
            
        print('"optimal control" gets %f'%(cost2go_std))
        print('"distubed control" gets %f'%(cost2go_disturb))
        

        all_cost_std.append(cost2go_std)
        all_cost_disturb.append(cost2go_disturb)

        ctrl.next_trial()
        traj.next_trial()

    sum_std = sum(all_cost_std)
    sum_disturb = sum(all_cost_disturb)
    print('Test %d trials'%(trial_cnt))
    print('"optimal control" gets %f'%(sum_std))
    print('"distubed control" gets %f'%(sum_disturb))
    if sum_std > sum_disturb:
        print('sum_std > sum_disturb')
    elif sum_std == sum_disturb:
        print('sum_std == sum_disturb')
    else:
        print('sum_std < sum_disturb')


