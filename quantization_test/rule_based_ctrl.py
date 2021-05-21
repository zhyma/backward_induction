import numpy as np
import csv
import sys
from sim_tool.py_sim import Vehicle

class a_ctrl():
    def __init__(self, gtr):
        self.dc_last = -1
        self.gtr = gtr
        self.est_dc_ = 0
        self.est_vc = 0
        self.est_vc_ = 0
        return

    def reset(self):
        self.dc_last = -1
        self.est_dc_ = 0
        self.est_vc = 0
        self.est_vc_ = 0
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

        # by using the safety distance constraint, calculate the acceleration
        # d <= dc-v*t_ttc -3:
        ak1 = 0
        for uk in range(a_n)[::-1]:
            d_, v_ = self.gtr.physical(self.gtr.a_list[uk])
            if d_ <= self.est_dc_ - v_*t_ttc - 3:
                ak1 = uk
                break

        # print("ak1 = %d, "%(ak1))

        # print("d2tl %f, rl_start %f, rl_end %f, d %f, t %d"%(d2tl, self.gtr.rl_start, self.gtr.rl_end, d, t))
        # by using the red-light safety constraint, calculate the acceleration
        #stage 1
        ak2 = 30
        if d < d2tl and t+dt >= self.gtr.rl_start and t < self.gtr.rl_end:
            # print("test red light")
            ak2 = 0
            for uk in range(a_n)[::-1]:
                # stage2: d2tl - d_ + 0.01 > 0.5*(v_**2)/(-a_min)
                d_, v_ = self.gtr.physical(self.gtr.a_list[uk])
                if d2tl - d_ + 3 > 0.5*(v_**2)/(-a_min):
                    ak2 = uk
                    break

        # print("ak2 = %d"%(ak2))

        # find the one fulfills both
        if ak1 < ak2:
            ak = ak1
        else:
            ak = ak2

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