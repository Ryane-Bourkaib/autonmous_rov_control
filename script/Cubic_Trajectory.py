#!/usr/bin/env python
import math
import numpy as np
import matplotlib.pyplot as plt
from alpha_beta_gamma_filter import alpha_beta_gamma_filter  
 
#STEP 1 : ROV Trajectory : Cubic trajectory

def Rov_Trajectory(Z_init, Z_final, t, t_final) :

    a_2 = 3 * (Z_final - Z_init) / t_final**2       #Polynomial parameter 
    a_3 = -2 * (Z_final - Z_init) / t_final**3      #Polynomial parameter
    
    if (t < t_final) :
        Z_desired = Z_init + a_2 * t**2 +a_3 * t**3             #Desired depth trajectory(m)
        Zd_desired =  Z_init + 2 * a_2 * t + 3 * a_3 * t**2     #Desired vertical speed (m/s)
    elif (t >= t_final) :
        Z_desired = Z_final
        Zd_desired = 0
        
    return  Z_desired, Zd_desired

#Ploting : 
#Initialisation 
t_final = 20
Z_init = 0
Z_final = 0.5
Z_desired, Zd_desired = np.zeros((2, t_final))  
 
# for t in range(t_final):
#     Z_desired[t], Zd_desired[t] = Rov_Trajectory(Z_init , Z_final, t, t_final) 

# print("Zd_desired", Z_desired.shape) 
# t = np.arange(20)
# plt.figure(2)
# plt.plot(t, Zd_desired)
# plt.xlabel("Time (s)")
# plt.ylabel("Depth (m) or vertical speed (m/s)")
# plt.title("curves of depth/ vertical speed")
# plt.show()


  
