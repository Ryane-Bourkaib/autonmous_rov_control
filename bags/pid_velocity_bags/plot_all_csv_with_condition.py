import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

### Set your path to the folder containing the .csv files
PATH = './bags/' # Use your path

### Fetch all files in path
fileNames = os.listdir(PATH)

### Filter file name list for files ending with .csv
fileNames = [file for file in fileNames if '.csv' in file]

# start index
idx = 0
### Loop over all files
for file in fileNames:

    # if file.find("_g_") != -1:  #skip files with this sub string
    #     continue
    if file.find("no_wall_2_Depth") == -1:    #skip files without this sub string
        continue

    ### Read .csv file and append to list
    df_d = pd.read_csv(PATH + file, index_col = None)
    df_d['%time'] = (df_d['%time'] - df_d['%time'].iloc[0]) / 1e6   #nano second to ms

    df_pwm = pd.read_csv(PATH + file.replace("_Depth", "_PWM"), index_col = None).drop(columns=['field.channels0','field.channels1','field.channels3','field.channels4','field.channels5','field.channels6','field.channels7'])
    df_pwm['%time'] = (df_pwm['%time'] - df_pwm['%time'].iloc[0]) / 1e6 #nano second to ms
    # bias = np.average (np.array(df['field.data'].tolist())[:5])
    # bias = 0
    # print (bias)
    # Y_d = np.array(df_d['field.data'].tolist())
    # Y_pwm = np.array(df_pwm['field.channels2'].tolist())
    # X = (np.arange(len(Y)) + idx + 10)
    # idx += len(Y) + 10

    # print(len(X))
    # print(len(Y))
    # print(idx)
    # break
    
    ### Create line for every file
    # plt.plot( df_d, label=file.replace("depth_control_12v_", "").replace(".csv", "").replace("_no_wall", "").replace("kp_p", "kp")+f', g={0}'+f', Ki={0.001}')
    # plt.plot( X, Y, label=file.replace("depth_control_12v_", "").replace(".csv", "")+"-bias-"+str(bias)[:5])
    # plt.rc('legend',fontsize='medium')
    

    fig, ax1 = plt.subplots()

    color = 'tab:red'
    ax1.set_xlabel('Time (milli second)')
    ax1.set_ylabel('Depth (m)', color=color)
    ax1.plot(df_d['%time'],df_d['field.data'], color=color, label=file.replace("depth_control_12v_", "").replace(".csv", "").replace("_Depth", "").replace("_no_wall_2", "").replace("kp_p", "kp"))
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

    color = 'tab:blue'
    ax2.set_ylabel('PPM', color=color)  # we already handled the x-label with ax1
    ax2.plot(df_pwm['%time'],df_pwm['field.channels2'], color=color)
    ax2.tick_params(axis='y', labelcolor=color)

fig.tight_layout()  # otherwise the right y-label is slightly clipped

### Generate the plot
# plt.ylabel("Depth(m)")
# plt.xlabel("Time Steps")
plt.legend(loc="best")
# plt.xlim(0, 37250)
# plt.ylim(-0.68, 0.5)
ax1.axhline(0.5,color='grey',linestyle="--") # y = 0
plt.grid()
plt.show()