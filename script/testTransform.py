#!/usr/bin/env python
import numpy as np
from transform import *

t = np.array([0,0,0])
r = np.array([0,90,90])

rRc = rotXYZ(r[0],r[1],r[2])
#print(rRc)
#print(isRot(rRc,10))

print ('rxc', rRc.dot([1,0,0])) 
print ('ryc', rRc.dot([0,1,0])) 
print ('rzc', rRc.dot([0,0,1])) 

#mat = skew_vec(t)
#print type (mat)
#print  (mat)

rMc = homogenousMatrix(t[0],t[1],t[2],r[0],r[1],r[2])
rVc = velocityTwistMatrix(t[0],t[1],t[2],r[0],r[1],r[2])


vcam_vs = np.array([1,2,3,0,0,0])
print 'Visual servoing : vcam =', vcam_vs
vrobot = rVc.dot(vcam_vs)
    
#print 'rVc', rVc
#print 'rMc', homogenousMatrix(rtc[0],rtc[1],rtc[2],rrc[0],rrc[1],rrc[2])
print 'Then vrobot = rVc * vcam = ', vrobot
