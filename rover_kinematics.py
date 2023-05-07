#!/usr/bin/python3
import math 

class Kinematics : 
    def __init__(self,linearVel,angularVel,rockerAngel,bogiAngle,frontWheelCentDist,verticalDisFront,wheelSelector):
        self.v = linearVel 
        self.omega = angularVel
        self.rho = rockerAngel 
        self.beta = bogiAngle 
        self.k4 = frontWheelCentDist
        self.k5 = verticalDisFront 
        self.b = wheelSelector  
    
    def frontWheel (self):
        self.xc1 = self.k4 * cos(-1*(-1)**(self.b * self.rho)) + self.k5 * sin(-1*(-1)**(self.wheelSelector * self.rho))
        
        
        
