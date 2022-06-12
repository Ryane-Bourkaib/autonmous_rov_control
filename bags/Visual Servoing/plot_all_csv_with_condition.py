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

file = 'bonus_dist.csv'
PATH = ''

### Read .csv file and append to list
df_d = pd.read_csv(PATH + file, index_col = None)
df_d['%time'] = (df_d['%time'] - df_d['%time'].iloc[0]) / 1e6   #nano second to ms

df_pwm = pd.read_csv(PATH + file.replace("_dist", "_thrusters"), index_col = None).drop(
                                                    columns=['field.channels0','field.channels1'])
df_pwm['%time'] = (df_pwm['%time'] - df_pwm['%time'].iloc[0]) / 1e6 #nano second to ms



fig, ax1 = plt.subplots()

color1 = 'black'
ax1.set_xlabel('Time (milli second)')
ax1.set_ylabel('Distance (mm)', color=color1)
ax1.plot(df_d['%time'],df_d['field0'], color=color1, label='Obs_distance') #, label=file.replace("depth_control_12v_", "").replace(".csv", "").replace("_Depth", "").replace("_no_wall_2", "").replace("kp_p", "kp"))
ax1.tick_params(axis='y', labelcolor=color1)

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:blue'
ax2.set_ylabel('PWM', color=color)  # we already handled the x-label with ax1
# Heave , Yaw ,Surge, Sway
ax2.plot(df_pwm['%time'],df_pwm['field.channels2'], color='crimson', label='Heave',linewidth=0.5)
ax2.plot(df_pwm['%time'],df_pwm['field.channels3'], color='blue', label='Yaw', linewidth=0.5)
ax2.plot(df_pwm['%time'],df_pwm['field.channels4'], color='green', label='Surge')
ax2.plot(df_pwm['%time'],df_pwm['field.channels5'], color='orange', label='Sway',linewidth=0.25)
# ax2.tick_params(axis='y', labelcolor=color)


ax1.legend()
ax2.legend()

fig.tight_layout()  # otherwise the right y-label is slightly clipped

### Generate the plot
# plt.ylabel("Depth(m)")
# plt.xlabel("Time Steps")
# plt.legend(loc="best")
# plt.xlim(0, 37250)
# plt.ylim(-0.68, 0.5)
# ax1.axhline(0.5,color='grey',linestyle="--") # y = 0
plt.grid()
plt.show()