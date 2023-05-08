#!/usr/bin/python3
import math 
from math import * 

class Diffrential:
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
        
        