from numpy.core.defchararray import array
import pandas as pd
from scipy.optimize import curve_fit
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib


font = {'weight': 'bold',
        'size': 22}

matplotlib.rc('font', **font)

def linear_eq(x, m):
    global c1
    return m*x + c1
    
def linear_eq_below(x, m):
    global c2
    return m*x + c2

def compute_pwm(f, m, c):
    return m*f + c
file_name = 'T200.xls'
sheet_name = '14 V'
file_path = os.path.abspath(os.getcwd()) + '/data/' + file_name
data = pd.read_excel(file_path, sheet_name=sheet_name)

# print(data.loc[data[' PWM (µs)'] >= 1500])
# print(data.loc[(data[' PWM (µs)'] >= 1500) & (data[' Force (Kg f)'] > 0)])

data_above_1500 = data.loc[(data[' Force (Kg f)'] > 0)]
data_below_1500 = data.loc[(data[' Force (Kg f)'] < 0)]
# print(data_below_1500)

# find the thrust shift
c1 = data_above_1500[' PWM (µs)'].array[0]
print(f"c1 = {c1}")
c2 = data_below_1500[' PWM (µs)'].array[-1]
print(f"c2 = {c2}")

# fit the +ve thrust
y_above = np.array(data_above_1500[' PWM (µs)'])
x_above = np.array(data_above_1500[' Force (Kg f)'])
result_above = curve_fit(linear_eq, x_above, y_above, p0=[0])

# fit the -ve thrust
y_below = np.array(data_below_1500[' PWM (µs)'])
x_below = np.array(data_below_1500[' Force (Kg f)'])
result_below = curve_fit(linear_eq_below, x_below, y_below, p0=[0])

# plot the data
y_all = np.array(data[' PWM (µs)'])
x_all = np.array(data[' Force (Kg f)'])
print('slope for positive thrust is: ', result_above[0][0])
print('slope for negative thrust is: ', result_below[0][0])
plt.plot(x_above, linear_eq(x_above, result_above[0][0]), linewidth=4)
plt.plot(x_below, linear_eq_below(x_below, result_below[0][0]), linewidth=4)
plt.plot(x_all, y_all, '--', linewidth=4)
plt.xlabel("Thrust Force (Kgf.)", **font)
plt.ylabel("PWM", **font)
plt.title(f"PWM vs Thrust(t200) ({sheet_name})", **font)
plt.grid()
plt.legend(["fitted +ve thrust", "fitted -ve thrust", "non-linear"])
plt.annotate(f'+ve thrust start at {c1}', xy=(0, c1),  xycoords='data',
            xytext=(0.8, 0.95), textcoords='axes fraction',
            arrowprops=dict(facecolor='black'),
            horizontalalignment='right', verticalalignment='top',
            bbox=dict(boxstyle="round", fc="w"))
plt.annotate(f'-ve thrust start at {c2}', xy=(0, c2),  xycoords='data',
             xytext=(0.8, 0.2), textcoords='axes fraction',
             arrowprops=dict(facecolor='black'),
             horizontalalignment='right', verticalalignment='top',
             bbox=dict(boxstyle="round", fc="w")
             )
plt.show()

required_thrust = 1.5/4
required_pwm = compute_pwm(required_thrust, result_above[0][0], c1)
print(f'Requireed PWM for {required_thrust} thrust is {required_pwm}')


