import matplotlib.pyplot as plt
import numpy as np

f = open("output/front_car_data.csv","r") 

n = 10000
for i in range(n):
    d = []
    while (True):
        line = f.readline()
        if 'end' in line:
            break
        d.append(np.fromstring(line, sep=',')[0])
    
    plt.plot(list(range(len(d))), d, 'g', alpha=0.1)
    
plt.show()
