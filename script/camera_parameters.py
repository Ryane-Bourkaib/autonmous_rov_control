#!/usr/bin/env python
import numpy as np


#camera parameters
u0 = 341
v0 = 258
lx = 455
ly = 455
kud =0.00683 
kdu = -0.01424     
    

def convert2meter(pt,u0,v0,lx,ly):
    return (pt[0]-u0)/lx, (pt[1]-v0)/ly

def convertListPoint2meter (points):
    global u0,v0,lx, ly
    
    if(np.shape(points)[0] > 1):
        n = int(np.shape(points)[0]/2)
        point_reshaped = (np.array(points).reshape(n,2))
        point_meter = []
        for pt in point_reshaped:
            pt_meter = convert2meter(pt,u0,v0,lx,ly)
            point_meter.append(pt_meter)
        point_meter = np.array(point_meter).reshape(-1)
        return point_meter
