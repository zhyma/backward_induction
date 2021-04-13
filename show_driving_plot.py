import matplotlib.pyplot as plt
import numpy as np
import os

from sim_tool.py_sim import find_all

curr_dir = os.getcwd() + '/output/'
files = find_all('front_car_data', curr_dir)

print(files[1])

f = open(files[1],'r')
line = f.readline()
param = line.split(',')
d2tl = float(param[0].split('=')[1])
rl_start = float(param[1].split('=')[1])/2

data = True
while data:
    d = []
    line = ''
    while data:
        line = f.readline()
        if 'end' in line:
            break
        if len(line) == 0:
            data = False
            break
        d.append(np.fromstring(line, sep=',')[0])
    if data == True:
        plt.plot(list(range(len(d))), d, 'g', alpha=0.5)
    
plt.axline((rl_start, d2tl),slope=0, color='black', linestyle=':')
plt.axline((rl_start, d2tl),slope=np.inf, color='black', linestyle=':')


plt.xlabel('step')
plt.ylabel('distance (m)')
plt.show()
