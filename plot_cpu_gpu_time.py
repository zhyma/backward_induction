# Import the necessary libraries to read 
# dataset and work on that 
import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt 

# Make the dataframe for evaluation on Errorbars 
df = pd.DataFrame({ 
	'Grain': [10, 15, 20, 50, 100] + [10, 15, 20, 50, 100], 
	'mean': [0.0170456, 0.079487, 0.2248316, 7.898748, 124.157] + [0.0957212, 0.0838592, 0.0904254, 0.1719228, 1.286828], 
	'quality': ['CPU average']*5 + ['GPU average']*5, 
	'std': [0.000592651, 0.001141872, 0.003034391, 0.008923736, 0.437862421] + [0.011953249, 0.00692039,0.009899268, 0.003762348, 0.018758001]}) 

print(df)


fig, ax = plt.subplots() 
  
for key, group in df.groupby('quality'): 
    group.plot('Grain', 'mean', yerr='std', marker='x',
               label=key, ax=ax) 
  
plt.show()