#!/usr/bin/python3
import math 
from math import * 

class Kinematics : 
    def __init__(self,v,omega,rho,beta,k4,k5,wheelConf,l2,l3,k7,k8):
        self.v = v 
        self.omega = omega
        self.rho = rho 
        self.beta = beta 
        self.k4 = k4
        self.k5 = k5 
        self.b = wheelConf   
        self.l2 = l2
        self.l3 = l3  
        self.k7 = k7 
        self.k8 = k8 
            
    def centerDistCalc (self):
        self.xc1 = self.k4 * cos((-1*(-1)**(self.b) )* self.rho) + self.k5 * sin((-1*(-1)**(self.b) )* self.rho)
        self.xc3 = self.l2 * cos((-1*(-1)**(self.b) )* self.rho) + self.l3 * sin ((-1*(-1)**(self.b))* self.rho) - ((self.k7 * cos(self.beta)) -(self.k8)* sin(self.beta))
        self.xc5 = self.l2 * cos((-1*(-1)**(self.b) )* self.rho) + self.l3 * sin ((-1*(-1)**(self.b))* self.rho) + ((self.k7 * cos(self.beta)) -(self.k8)* sin(self.beta))
    
    
        
        
