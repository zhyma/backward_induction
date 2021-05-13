import numpy as np
import csv
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from scipy.stats import truncnorm
from math import isclose

## Generates the front car behavior, saves to output/front_car_data.csv

# gs stands for the sigma of the gaussian distribution
gs = 0.5
a_step = 0.4

class Vehicle():
    def __init__(self, d2tl, rl_start, dt, v_min, v_max, a_min, a_max):
        self.d2tl = d2tl
        self.rl_start = rl_start
        self.dt = dt
        
        self.d = 0
        self.t = 0
        # boudnary of velocity is [0, v_bound]
        self.v_min = 0.0
        self.v_max = 18.0
        self.a_min = -4.0
        self.a_max = 2.0
        # mode 1: accelerate to pass the red light, mode 0: decelerate to stop before the red light
        self.mode = 0
        self.a = 0
        self.intention = 1

        # a_list is only used for 
        # searching the requirement of stop before the red light
        self.a_n = 30
        self.a_list = self.discretize(a_min, a_max, self.a_n)
        
        return

    def reset(self):
        self.t = 0
        self.d = 0
        self.v = 0
        self.a = 0
        self.mode = 0
        self.intention = 1
        return

    def draw_a_number(self):
        lower = 0
        upper = 0
        mu = 0
        a = self.a
        a_min = self.a_min
        a_max = self.a_max
        t = self.t
        d2tl = self.d2tl
        d = self.d
        v = self.v
        dt = self.dt

        # must accelerate
        if self.mode == 1:
            if self.d > self.d2tl:
                upper = 1
                lower = 0
                mu = (upper+lower)/2
            else:
                lower = 2*(d2tl+3-d-v*(rl_start-t))/((rl_start-t+0.00001)**2)
                if lower >= a_max:
                    return a_max
                upper = a_max
                mu = (upper+lower)/2

        # to stop (will brake)
        # There is a maximium acceleration (when decelerating)
        else:
            if d2tl-d < 3:
                if v < 0.1:
                    self.v = 0
                    return 0
                elif v < -self.dt*a_min:
                    return -v/self.dt
                
            # go as close as possible
            else:
                upper = self.rl_constraint()
                if upper <= a_min:
                    return a_min
                lower = upper - 2
                if lower <= a_min:
                    lower = a_min
                mu = (upper+lower)/2

        sigma = 1

        # Truncated Gaussin distribution
        x = truncnorm((lower - mu) / sigma, (upper - mu) / sigma,\
                    loc=mu, scale=sigma)
        
        return x.rvs(1)[0]

    def discretize(self, min, max, cnt):
        val_list = []
        for i in range(cnt):
            val = min + (max-min)/float(cnt-1)*i
            val_list.append(val)
        return val_list

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

    def rl_constraint(self):
        # if current state hits the constraint, apply penalty

        d = self.d
        v = self.v
        
        d2tl = self.d2tl
        a_min = self.a_min

        # traffic light condition
        # if d < d2tl and self.rl_state:
            # check before the red light, if not enough to brake
        if d2tl - 3 - d + 0.01 < 0.5*(v**2)/(-a_min):
            # hit the constraint
            return a_min
        # if in front of a red light, check the acceleration
        i = self.a_n-2
        while i >= 0:
            d_, v_ = self.physical(self.a_list[i])
            if d2tl -3 - d_ + 0.01 < 0.5*(v_**2)/(-a_min):
                i -= 1
                continue
            else:
                break

        # print('a upper bound:%.2f'%(self.a_list[i+1]))
        return self.a_list[i+1]

    def sim_step(self, a):
        d_, v_ = self.physical(a)
        self.d = d_
        self.v = v_
        self.a = a
        self.t += self.dt
        return self.d, self.v

    def vehicle_ctrl(self):
        d = self.d
        v = self.v

        a = self.draw_a_number()

        if v> 0.1 and a >= 0:
            self.intention = 1
        else:
            self.intention = 0

        return a, self.intention

def simulate(d2tl, rl_start, iter):
    rl_end = 60

    trajs = []

    print("%d, %d, %d"%(d2tl, rl_start, rl_end))
    with open('output/front_car_data.csv', mode='w') as csv_file:
        writer = csv.writer(csv_file, delimiter=',')
        writer.writerow(['d2tl='+str(d2tl), 'rl_start='+str(rl_start), 'rl_end='+str(rl_end)])
        d_final = []
        dt = 2
        gtr = Vehicle(d2tl, rl_start, dt, 0, 18, -4.0, 2.0)
        for i in range(iter):
            v = 8
            # starting at a position that guarantees 
            d = 40
            
            # d2tl, dt, rl_start, rl_end, v_min, v_max, a_min, a_max
            gtr.reset()
            gtr.d = float(d)
            gtr.v = float(v)
            trip = []

            if d > 45:
                gtr.mode = 1
            else:
                gtr.mode = 0
            
            for k in range(13):
                # a, intention = gtr.vehicle_ctrl()
                if gtr.mode == 1:
                    a, intention = gtr.vehicle_ctrl()
                    # a, intention = 2, 1
                else:
                    a, intention = gtr.vehicle_ctrl()
                    # a, intention = -4, 0


                writer.writerow([format(gtr.d, '.2f'), format(gtr.v, '.2f'), format(a, '.2f'), intention])
                trip.append([gtr.d, gtr.v, a, intention, gtr.mode])

                d, v = gtr.sim_step(a)
                    
            trajs.append(trip)
            writer.writerow(["end"])
            d_final.append(d)
            if i%(iter/10)==0:
                print(f"{i/(iter/10):.0f}0%...")
    
    return trajs

if __name__ == "__main__":
    # n = 10**6
    n = 50000
    d2tl = 150
    rl_start = 10
    trajs = simulate(d2tl, rl_start, n)
    for i in trajs:
        t = [x[0] for x in i]
        if i[0][4] == 1:
            plt.plot(list(range(len(t))), t, 'g', alpha=0.2)
        else:
            plt.plot(list(range(len(t))), t, 'r', alpha=0.2)
    
    plt.axline((rl_start/2, d2tl),slope=0, color='black', linestyle=':')
    plt.axline((rl_start/2, d2tl),slope=np.inf, color='black', linestyle=':')
    n_step = len(t)
    plt.yticks(fontsize='x-large')
    plt.xticks([i for i in range(n_step)], [str(i*2) for i in range(n_step)], fontsize='x-large')
    plt.xlabel('Time (s)', fontsize='x-large')
    plt.ylabel('Distance (m)', fontsize='x-large')
    plt.show()
    # statistics()
   