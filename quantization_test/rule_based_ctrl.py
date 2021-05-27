import numpy as np
import csv
import sys
from sim_tool.py_sim import Vehicle
from math import sqrt

class a_ctrl():
    def __init__(self, gtr):
        self.dc_last = -1
        self.gtr = gtr
        self.est_dc_ = 0
        self.est_vc = 0
        self.est_vc_ = 0
        self.stop = False
        return

    def reset(self):
        self.dc_last = -1
        self.est_dc_ = 0
        self.est_vc = 0
        self.est_vc_ = 0
        self.stop = False
        self.gtr.reset()
        return

    # dc: front vehicle position
    def gen_ctrl(self, dc, i):
        t_ttc = 3
        a_min = self.gtr.a_min
        a_n = len(self.gtr.a_list)

        d = self.gtr.d
        v = self.gtr.v
        d2tl = self.gtr.d2tl
        t = self.gtr.t
        # estimate the next position of the front car
        ak = 0
        dt = self.gtr.dt

        if self.dc_last < 0:
            # no last dc data, cannot estimate the front car velocity
            self.est_vc = 0
            self.est_dc_ = dc
        else:
            # estimate the front car velocity
            # if dc - self.dc_last < -0.5*(-2)*(dt**2):
            #     est_v_front = 0
            # else:
            #     est_v_front = (dc-self.dc_last)/dt+0.5*(-2)*dt
            self.est_vc = (dc - self.dc_last)/dt + a_min*dt/2 - 0.2
            if self.est_vc < 0:
                self.est_vc = 0

            # estimate the front car acceleration
            if i == 0:
                ac = a_min
            else:
                ac = 0

            self.est_dc_, self.est_vc_ = self.gtr.physical(ac, d=dc, v=self.est_vc)

        if d < d2tl and t >= self.gtr.rl_start:
            self.stop = True

        if t > self.gtr.rl_end:
            self.stop = False

        # by using the safety distance constraint, calculate the acceleration
        # d <= dc-v*t_ttc -3:
        ak = 0
        for uk in range(a_n)[::-1]:
            d_, v_ = self.gtr.physical(self.gtr.a_list[uk])
            if d_ <= self.est_dc_ - v_*t_ttc - 3:
                ak = uk
                break

        # find a even smaller control
        a_val = self.gtr.a_list[ak]
        if a_val > 0:
            ak_upper = ak
            ak = 20
            a_val = a_val * (1-sqrt(self.gtr.v/self.gtr.v_max))*0.8
            # ak, a_val = self.gtr.find_closest(a_val, self.gtr.a_list)
            for uk in range(a_n)[ak_upper::-1]:
                if self.gtr.a_list[uk] < a_val:
                    ak = uk
                    break

        # print("ak1 = %d, "%(ak1))

        # # print("d2tl %f, rl_start %f, rl_end %f, d %f, t %d"%(d2tl, self.gtr.rl_start, self.gtr.rl_end, d, t))
        # by using the red-light safety constraint, calculate the acceleration
        #stage 1
        # if d < d2tl and t+dt >= self.gtr.rl_start and t < self.gtr.rl_end:
        if self.stop == True:
            # print("test red light")
            ak_upper = ak
            ak = 0
            for uk in range(a_n)[ak_upper::-1]:
                # stage2: d2tl - d_ + 0.01 > 0.5*(v_**2)/(-a_min)
                d_, v_ = self.gtr.physical(self.gtr.a_list[uk])
                if d2tl - d_ + 0.1 > 0.5*(v_**2)/(-a_min):
                    # print("d_=%.2f, v_=%.2f"%(d_,v_))
                    # print(d2tl - d_ + 0.1 - 0.5*(v_**2)/(-a_min))
                    ak = uk
                    break


        self.dc_last = dc
        return ak

def ctrl_seq(traj, gtr):
    gtr.reset()
    ctrller = a_ctrl(gtr)
    # ctrller.reset()
    ctrl_seq = []
    for i in traj:
        ak = ctrller.gen_ctrl(i[0], i[1])
        a = gtr.a_list[ak]
        gtr.step_forward(a)
        ctrl_seq.append(ak)

    return ctrl_seq