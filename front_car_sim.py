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
    def __init__(self, d2tl, dt, v_min, v_max, a_min, a_max):
        self.d2tl = d2tl
        self.rl_state = False
        self.dt = dt
        
        self.d = 0
        # boudnary of velocity is [0, v_bound]
        self.v_min = 0.0
        self.v_max = 18.0
        self.a_min = -4.0
        self.a_max = 2.0
        # mode 0: cruising, mode -1: to stop, mode 1: accelerate
        self.mode = 0
        self.a = 0
        self.intention = 1

        # a_list is only used for 
        # searching the requirement of stop before the red light
        self.a_n = 30
        self.a_list = self.discretize(a_min, a_max, self.a_n)
        
        return

    def reset(self):
        self.rl_state = False
        self.t = 0
        self.d = 0
        self.v = 0
        self.a = 0
        self.mode = 0
        self.intention = 1
        return

    def draw_a_number(self, upper_bound = 2):
        lower = 0
        upper = 0
        mu = 0
        a = self.a
        a_min = self.a_min
        a_max = self.a_max

        # must accelerate
        if self.mode == 1:
            if a <= 0:
                lower = 0
                mu = gs
                upper = gs*2
            else:
                mu = a + a_step
                lower = mu - gs
                upper = mu + gs
                if lower < 0:
                    lower = 0
                if upper > a_max:
                    upper = a_max

        # to stop (will brake)
        # There is a maximium acceleration (when decelerating)
        elif self.mode == 0:
            mu = 0
            lower = mu - gs
            upper = mu + gs
        else:
            # mode == -1
            upper = upper_bound
            if a - a_step > upper_bound:
                # need a brand new mu
                lower = upper_bound - gs*2
                if lower < a_min:
                    lower = a_min
                mu = (upper+lower)/2
            elif a - a_step > a_min:
                mu = a - a_step
                lower = mu - gs
                if lower < a_min:
                    lower = a_min
            else:
                lower = a_min
                mu = (upper+lower)/2

        # print("%f, %f, %f"%(lower, mu, upper))

        if upper < a_min+0.01:
            return a_min

        sigma = 0.3

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

    def rl_constraint(self):
        # if current state hits the constraint, apply penalty

        d = self.d
        v = self.v
        
        d2tl = self.d2tl
        a_min = self.a_min

        # traffic light condition
        # if d < d2tl and self.rl_state:
            # check before the red light, if not enough to brake
        if d2tl - d + 0.01 < 0.5*(v**2)/(-a_min):
            # hit the constraint
            return a_min
        # if in front of a red light, check the acceleration
        i = self.a_n-2
        while i >= 0:
            d_, v_ = self.physical(self.a_list[i])
            if d2tl - d_ + 0.01 < 0.5*(v_**2)/(-a_min):
                i -= 1
                continue
            else:
                break

        print('a upper bound:%.2f'%(self.a_list[i+1]))
        return self.a_list[i+1]

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

    def sim_step(self, a):
        d_, v_ = self.physical(a)
        self.d = d_
        self.v = v_
        self.a = a
        return self.d, self.v

    def vehicle_ctrl(self):
        d = self.d
        v = self.v

        if (self.rl_state and d < self.d2tl):
            if v > 0.5:
                a_upper = self.rl_constraint()
                self.mode = -1
                a = self.draw_a_number(a_upper)
            else:
                self.v = 0
                a = 0
        else:
            # accelerating
            if v < self.v_max - 3:
                self.mode = 1
                a = self.draw_a_number()
            # cruising
            else:
                self.mode = 0
                a = self.draw_a_number()
        
        if a > self.a_max:
            a = self.a_max
        if a < self.a_min:
            a = self.a_min

        if v> 0.1 and a >= 0:
            self.intention = 1
        else:
            self.intention = 0

        return a, self.intention

def simulate(d2tl, rl_start, iter):
    # root = ET.parse('config.xml').getroot()
    # # distance to traffic light
    # d2tl = int(root[0].text)
    # # time to redlight
    # rl_start = int(root[1].text)
    # rl_end = int(root[2].text)

    # # meters
    # d2tl = 170
    # # seconds
    # rl_start = 8
    # seconds
    rl_end = 60

    trajs = []

    print("%d, %d, %d"%(d2tl, rl_start, rl_end))
    with open('output/front_car_data.csv', mode='w') as csv_file:
        writer = csv.writer(csv_file, delimiter=',')
        writer.writerow(['d2tl='+str(d2tl), 'rl_start='+str(rl_start), 'rl_end='+str(rl_end)])
        d_final = []
        for i in range(iter):
            t = 0
            v = np.random.uniform(8,12,1)
            # starting at a position that guarantees 
            d = v*3 + 3 + np.random.uniform(0, 20, 1)
            
            dt = 2
            
            # d2tl, dt, rl_start, rl_end, v_min, v_max, a_min, a_max
            gtr = Vehicle(d2tl, dt, 0, 18, -4.0, 2.0)
            gtr.d = float(d)
            gtr.v = float(v)
            trip = []
            for k in range(13):
                if (t >= rl_start):
                    gtr.rl_state = True
                    # print("red  : ", end="")
                
                a, intention = gtr.vehicle_ctrl()

                writer.writerow([format(gtr.d, '.2f'), format(gtr.v, '.2f'), format(a, '.2f'), intention])
                trip.append([gtr.d, gtr.v, a, intention])

                print('at time %d, to red light:%.2f'%(t, d2tl-gtr.d))

                d, v = gtr.sim_step(a)
                t += dt
                # print("[%f, %f, %f, %d]"% (gtr.d, gtr.v, a, intention))
                    
            trajs.append(trip)
            writer.writerow(["end"])
            print('next traj')
            d_final.append(d)
            if i%(iter/10)==0:
                print(f"{i/(iter/10):.0f}0%...")
    
    return trajs

# def statistics():
#      # position
#     pos = []
#     # distance between two states
#     ds = []
#     for i in range(mat.shape[0]):
#         for j in range(mat.shape[1]-1):
#             ds.append(mat[i,j+1,0] - mat[i,j,0])
#             pos.append(mat[i,j,0])
#         pos.append(mat[i,-1,0])

#     print(max(ds), end=', ')
#     print(min(ds))
#     print(max(pos), end=', ')
#     print(min(pos))
#     fig, axs = plt.subplots(1, 2, sharey=True, tight_layout=True)

#     # We can set the number of bins with the `bins` kwarg
#     axs[0].hist(pos, bins=353)
#     axs[1].hist(ds, bins=128)
#     plt.show()



if __name__ == "__main__":
    # n = 10**6
    n = 3
    d2tl = 170
    rl_start = 2
    trajs = simulate(d2tl, rl_start, n)
    for i in trajs:
        t = [x[0] for x in i]
        plt.plot(list(range(len(t))), t, 'g', alpha=0.5)
    
    plt.axline((rl_start/2, d2tl),slope=0, color='black', linestyle=':')
    plt.axline((rl_start/2, d2tl),slope=np.inf, color='black', linestyle=':')
    plt.xlabel('step')
    plt.ylabel('distance (m)')
    plt.show()
    # statistics()
   