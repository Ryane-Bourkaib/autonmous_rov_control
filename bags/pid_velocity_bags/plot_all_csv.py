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


    ### Read .csv file and append to list
    df = pd.read_csv(PATH + file, index_col = None)
    # bias = np.average (np.array(df['field.data'].tolist())[:5])
    bias = -0.1
    # print (bias)
    Y = np.array(df['field.data'].tolist()) - bias
    X = (np.arange(len(Y)) + idx + 10)
    idx += len(Y) + 10

    # print(len(X))
    # print(len(Y))
    # print(idx)
    # break
    
    ### Create line for every file
    plt.plot( X, Y, label=file.replace("depth_control_12v_", "").replace(".csv", "").replace("_no_wall", "").replace("kp_p", "kp"))
    # plt.plot( X, Y, label=file.replace("depth_control_12v_", "").replace(".csv", "")+"-bias-"+str(bias)[:5])
    plt.rc('legend',fontsize=7)
    # break

### Generate the plot
plt.legend(loc="best")
plt.xlim(0, 37250)
plt.ylim(-0.68, 0.5)
plt.axhline(-0.5,color='grey',linestyle="--") # y = 0
plt.grid()
plt.show()