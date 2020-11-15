# Import the necessary libraries to read 
# dataset and work on that 
import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt 

mat_warm = np.genfromtxt('gpu_time.csv', delimiter=',')[:,2:]
mat_cold = np.genfromtxt('gpu_time.csv', delimiter=',')[:,1]

# Make the dataframe for evaluation on Errorbars 
df = pd.DataFrame({ 
	'steps': [i*10 for i in range(1,11)] + [i*10 for i in range(1,11)], 
    'type': ['GPU average (with warmup)']*10 + ['GPU cold start']*10, 
	'mean': [np.mean(mat_warm[i]) for i in range(len(mat_warm))] + [mat_cold[i] for i in range(len(mat_cold))], 
	'std': [np.std(mat_warm[i]) for i in range(len(mat_warm))] + [0]*10}) 

print(df)


fig, ax = plt.subplots() 
  
for key, group in df.groupby('type'): 
    group.plot('steps', 'mean', yerr='std', marker='x',
               label=key, ax=ax, ylabel='time(Sec)') 
  
plt.show()