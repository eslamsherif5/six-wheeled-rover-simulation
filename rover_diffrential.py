#!/usr/bin/python3
import math 
from math import * 

class Diffrential:
    def __init__(self,v,omega,k3,k3bar):
        self.v = v 
        self.omega = omega
        self.k3 = k3 
        self.k3bar = k3bar 
        
    def dimensions (self):
        # ICR redius calcuation 
        self.r = self.omega / self.v 
        # Dimensions for the left side of the rover 
        self.l1 = self.r - self.k3
        self.l35 = self.r - self.k3 - self.k3bar 
        # Dimenstions for the right side of the rover 
        self.l2 = self.r + self.k3
        self.l46 = self.r - self.k3 + self.k3bar
    
    def velocities (self):
        # Velcoity calcuation for the rover 
        v1 = self.l1 * self.omega     
        v2 = self.l2 * self.omega
        v3 = self.l3 * self.omega
        v4 = self.l4 * self.omega
        v5 = self.l5 * self.omega
        v6 = self.l6 * self.omega
    