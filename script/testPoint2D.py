#!/usr/bin/env python
import math
import numpy as np
import random
import matplotlib.pyplot as plt

def order_point(previous_pts, current_pts):
    ordered_pts = []
    for ppt in previous_pts:
        index = np.argmin(np.linalg.norm(current_pts - ppt, axis=1)) 
        ordered_pts.append(current_pts[index])
    return ordered_pts

current_pts = []
previous_pts = []
ordered_pts = []

nb_pts = 50



for i in range(0,nb_pts):
    pt = 75+10*i , 300-20*i
    current_pts.append([pt[0],pt[1]]);
    

#copy the points and shuffle them
previous_pts = current_pts[:]#+ 0.01*np.ones(np.shape(current_pts)) 
print "begin"
print "current_pts",current_pts
print "previous_pts",previous_pts 


print previous_pts
random.shuffle(previous_pts)
print "after shuffle"
print "current_pts",current_pts
print "previous_pts",previous_pts 

#previous_pts += 0.01*np.ones(np.shape(previous_pts)) 
previous_pts += 2*np.random.rand(nb_pts,2)-1

print "after noise"
print "current_pts",current_pts
print "previous_pts",previous_pts 



ordered_pts =  order_point(previous_pts, current_pts)

print "ordered"
print "current_pts",current_pts
print "previous_pts",previous_pts  
print "ordered_pts",ordered_pts

plt.figure(1)
plt.plot(np.array(current_pts)[:,0],np.array(current_pts)[:,1],'ro')
plt.plot(np.array(previous_pts)[:,0],np.array(previous_pts)[:,1],'b*')
plt.plot(np.array(ordered_pts)[:,0],np.array(ordered_pts)[:,1],'gs')
plt.xlabel('u')
plt.ylabel('v')
plt.show()

plt.figure(2)
plt.plot(current_pts,'ro')
plt.plot(previous_pts,'b*')
plt.plot(ordered_pts,'gs')
plt.xlabel('point number')
plt.ylabel('u,v')
plt.show()
