#!/usr/bin/python3 
import matplotlib.pyplot as mp


class DiffrentialRover:
    def __init__(self,v,omega,k3,k3bar):
        self.v = v 
        self.omega = omega
        self.k3 = k3 
        self.k3bar = k3bar
        self.l1 = 0  
        self.l35 = 0 
        self.l2 = 0 
        self.l46 = 0 
    def dimensions (self):
        # ICR redius calcuation 
        self.r = self.omega / self.v 
        # Dimensions for the left side of the rover 
        self.l1 = self.r - self.k3
        self.l35 = self.r - self.k3 - self.k3bar 
        # Dimenstions for the right side of the rover 
        self.l2 = self.r + self.k3
        self.l46 = self.r + self.k3 + self.k3bar
    
    def velocities (self):
        self.dimensions()
        # Velcoity calcuation for the rover 
        v1 = self.l1 * self.omega     
        v2 = self.l2 * self.omega
        v3 = self.l35 * self.omega
        v4 = self.l46 * self.omega
        v5 = self.l35 * self.omega
        v6 = self.l46 * self.omega

        return [v1,v2,v3,v4,v5,v6]
        
v = DiffrentialRover (2,2,0.2,0.05)
vel = v.velocities()
for i in range(len(vel)):
    print('Rover Velcotiy v'+str(i)+ ': ' + str(vel[i]))

# mp.plot(vel)
# mp.show()