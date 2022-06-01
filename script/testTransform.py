#!/usr/bin/env python
import numpy as np
from transform import *

t = np.array([0,0,0])
r = np.array([0,0,0])
rRc = rotXYZ(r[0],r[1],r[2])
print("Roration",rRc)
print("Is R a rotation in SO(3) ? ",isRot(rRc,10))

# print the x,y,z axis transform
print ('x vector ---> ', rRc.dot([1,0,0])) 
print ('y vector --->', rRc.dot([0,1,0])) 
print ('z vector --->', rRc.dot([0,0,1])) 

#mat = skew_vec(t)
#print type (mat)
#print  (mat)

# this is the function to build an homogenousMatrix
rMc = homogenousMatrix(t[0],t[1],t[2],r[0],r[1],r[2])

#this is the function to build a twist matrix
rVc = velocityTwistMatrix(t[0],t[1],t[2],r[0],r[1],r[2])


vcam_vs = np.array([1,2,3,0,0,0])
print 'Visual servoing : vcam =', vcam_vs
vrobot = rVc.dot(vcam_vs)
    
print 'Then vrobot = rVc * vcam = ', vrobot
