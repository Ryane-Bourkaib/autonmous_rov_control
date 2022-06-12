import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

### Set your path to the folder containing the .csv files
# PATH = './' # Use your path

### Fetch all files in path
# fileNames = os.listdir(PATH)

### Filter file name list for files ending with .csv
# fileNames = [file for file in fileNames if '.csv' in file]
# file = ""

# # start index
# idx = 0
# ### Loop over all files
# for f in fileNames:

#     # if file.find("_g_") != -1:  #skip files with this sub string
#     #     continue
#     if f.find("bonus_dist") == 1:    #skip files without this sub string
#         file = f
#         break

#['surge_sway_servoing_error.csv', 'vx_vy_wz_servoing_error.csv']
file = 'surge_sway_servoing_error.csv'
PATH = './'

### Read .csv file and append to list
df_d = pd.read_csv(PATH + file, index_col = None)
df_d['%time'] = (df_d['%time'] - df_d['%time'].iloc[0]) / 1e9   #nano second to s

df_error16 = pd.read_csv(PATH + file, index_col = None).drop( columns=['%time','field.layout.data_offset'])
np_error16 = df_error16.to_numpy().reshape((1800, -1, 2))
print(np_error16.shape)
np_error16_n = np.linalg.norm(np_error16, axis=(1,2))
print(np_error16_n)


color = 'tab:red'
plt.xlabel('Time in seconds')
plt.ylabel('||Error||', color=color)
plt.plot(df_d['%time']+25,np_error16_n, color=color) #17.2
plt.tick_params(axis='y', labelcolor=color)




plt.tight_layout()  # otherwise the right y-label is slightly clipped

### Generate the plot
# plt.ylabel("Depth(m)")
# plt.xlabel("Time Steps")
# plt.legend(loc="best")
# plt.xlim(0, 37250)
# plt.ylim(-0.68, 0.5)
# ax1.axhline(0.5,color='grey',linestyle="--") # y = 0
plt.grid()
plt.show()