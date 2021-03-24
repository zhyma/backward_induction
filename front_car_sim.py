import numpy as np
import csv
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET

## Generates the front car behavior, saves to output/front_car_data.csv

def draw_a_number(lower, upper):
    mid = (lower+upper)/2.0
    while True:
        x = np.random.normal(mid, abs(upper - mid), 1)[0]
        if (x > lower) and (x < upper):
            return x

class Vehicle():
    def __init__(self, d2tl, dt, v_bound, a_bound):
        self.d2tl = d2tl
        self.dt = dt
        
        self.d = 0
        # boudnary of velocity is [0, v_bound]
        self.v_bound = [0, v_bound]
        self.v = draw_a_number(5, self.v_bound[1])
        self.a_bound = a_bound
        self.a = 0
        self.intention = 1
        return

    def make_decision(self, percentage):
        x = np.random.uniform(0, 1, 1)
        # if had decided to stop, keep press the brake
        if (self.intention == 0) and (self.d < self.d2tl):
            self.intention = 0
        # passed the light, accelerate
        elif (self.d > self.d2tl):
            self.intention = 1
        else:
            if x < percentage:
                self.intention = 0

            else:
                self.intention = 1

        return

    def sim_step(self):
        dt = self.dt
        d = self.d
        v = self.v
        v_b = self.v_bound[1]
        a = self.a
        if a > self.a_bound[1]:
            a = self.a_bound[1]
        if a < self.a_bound[0]:
            a = self.a_bound[0]

        # speed and acceleration is equal or greater than 0
        # or final speed not exceeding upper bound
        if (v + a*dt >= 0) and (v + a*dt <= v_b):
            d_ = d+v*dt+0.5*a*dt
            v_ = v+a*dt
        ## speed can't exceed upper bound
        elif (v+a*dt > v_b):
            d_ = d + (v+v_b)/2*((v_b-v)/a) + (dt-(v_b-v)/a)*v_b
            v_ = v_b
        ## speed can't be negative
        else:
            d_ = d-v**2/(2*a)
            v_ = 0

        self.d = d_
        self.v = v_
        return d_, v_

    # running at a constant speed
    def vehicle_ctrl(self, k):
        a_list = [0, 0, -1, -2, -1.5, -1]+[0]*25
        self.a = a_list[k]
        if self.a > 0:
            self.intention = 1
        else:
            self.intention = 0
        return self.a, self.intention

    # def vehicle_ctrl(self, rl):
    #     d = self.d
    #     v = self.v
    #     d2stop = self.d2tl - d

    #     # green light or has past the traffic light, accelerate
    #     if (rl == False) or (d > self.d2tl):
    #         if v < 10:
    #             self.a = draw_a_number(1.0, 2.0)
    #         if v >= 10:
    #             self.a = draw_a_number( .0, 1.0)
    #         self.intention = 1
    #     # red light, decellerate or accelerate?
    #     else:
    #         a_ref1 = -2.0
    #         a_ref2 = -4.0
    #         a_ref3 = self.a_bound[0]

    #         if (v == 0):
    #             self.a = 0
    #         # have enough distance to decelerate slowly
    #         elif d2stop >= abs(v**2/2/a_ref1):
    #             # print("slowly slow down")
    #             # 100% slow down
    #             self.intention = 0
    #             a = -v**2/d2stop
    #             self.a = draw_a_number(a-1.5, a)
    #         # distance only enough for decelerate hard
    #         elif d2stop >= abs(v**2/2/a_ref2):
    #             # 60% slow down
    #             # print("60% to slowly slow down")
    #             self.make_decision(0.8)
    #             if self.intention == 0:
    #                 a = -v**2/d2stop
    #                 self.a = draw_a_number(a-1.5, a)
    #             else:
    #                 self.a = draw_a_number(self.a_bound[1]-0.5, self.a_bound[1])
    #         # distance may not enough to stop, acceleration then
    #         elif d2stop >= abs(v**2/2/a_ref3):
    #             # 30% slow down
    #             # print("30% to slowly slow down")
    #             self.make_decision(0.3)
    #             if self.intention == 0:
    #                 self.a = self.a_bound[0]
    #             else:
    #                 self.a = draw_a_number(1.5, 2.0)
    #         # distance not enough to fully stop, full acceleration then
    #         else:
    #             # print("rush through the red")
    #             self.a = self.a_bound[1]
        
    #     if self.a > self.a_bound[1]:
    #         self.a = self.a_bound[1]
    #     if self.a < self.a_bound[0]:
    #         self.a = self.a_bound[0]

    #     return self.a, self.intention

def simulate(iter):
    # root = ET.parse('config.xml').getroot()
    # # distance to traffic light
    # d2tl = int(root[0].text)
    # # time to redlight
    # rl_start = int(root[1].text)
    # rl_end = int(root[2].text)

    d2tl = 100
    rl_start = 10
    rl_end = 60

    print("%d, %d, %d"%(d2tl, rl_start, rl_end))
    with open('output/front_car_data.csv', mode='w') as csv_file:
        writer = csv.writer(csv_file, delimiter=',')
        writer.writerow(['d2tl='+str(d2tl), 'rl_start='+str(rl_start), 'rl_end='+str(rl_end)])
        d_final = []
        for i in range(iter):
            t = 0
            # d = 54 + np.random.uniform(0, 60, 1)
            # v = np.random.uniform(0,18,1)
            d = 7
            v = 11
            
            dt = 2
            
            gtr = Vehicle(d2tl, dt, 18, [-4.0, 2.0])
            gtr.d = float(d)
            gtr.v = float(v)
            trip = []
            for k in range(24):
                if (t >= rl_start) and (t <= rl_end):
                    rl = True
                    # print("red  : ", end="")
                else:
                    rl = False
                    # print("green: ", end="")
                
                a, intention = gtr.vehicle_ctrl(k)
                d, v = gtr.sim_step()
                t += dt
                # print("[%f, %f, %f, %d]"% (gtr.d, gtr.v, a, intention))
                if k >= 0:
                    writer.writerow([format(gtr.d, '.2f'), format(gtr.v, '.2f'), format(a, '.2f'), intention])
                    trip.append([gtr.d, gtr.v, a, intention])

            states.append(trip)
            writer.writerow(["end"])
            # print("end")
            d_final.append(d)
            if i%(iter/10)==0:
                print(f"{i/(iter/10):.0f}0%...")
    # print("")
    print(max(d_final))

def statistics():
     # position
    pos = []
    # distance between two states
    ds = []
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]-1):
            ds.append(mat[i,j+1,0] - mat[i,j,0])
            pos.append(mat[i,j,0])
        pos.append(mat[i,-1,0])

    print(max(ds), end=', ')
    print(min(ds))
    print(max(pos), end=', ')
    print(min(pos))
    fig, axs = plt.subplots(1, 2, sharey=True, tight_layout=True)

    # We can set the number of bins with the `bins` kwarg
    axs[0].hist(pos, bins=353)
    axs[1].hist(ds, bins=128)
    plt.show()


if __name__ == "__main__":
    # n = 10**6
    n = 10**2
    states = []
    simulate(n)
    mat = np.array(states)
    # statistics()
   