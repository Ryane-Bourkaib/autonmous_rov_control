#!/usr/bin/env python


class Point2D:
    "2D point geometric class"

    
    def __init__(self,u=0,v=0):
        self.u = u
        self.v = v
        self.x = 0
        self.y = 0
        
    def __repr__(self):
        chaine= "x: {0} \n y: {1}".format(self.x, self.y)
        return chaine
    
    def convert2meter(self, lx, ly, u0, v0):
        self.x = (self.u-u0)/lx
        self.y = (self.v-v0)/ly
    
    def fromKeypoint(self, keypoint):
        self.u, self.v = keypoint.pt
        
    def convert2meter(self,u0,v0,lx,ly):
        if(lx!=0 and ly !=0):
            self.x = (self.u-u0)/lx
            self.y = (self.v-v0)/ly
        else :
            print("Error in convert2meter of class Point2D : lx and ly must be non zero integers")
            self.x = 0
            self.y = 0
