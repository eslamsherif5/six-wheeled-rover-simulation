#!/usr/bin/python3
import matplotlib.pyplot as mp
from math import *
import numpy as np
from tictoc import tic, toc


class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


class DiffrentialRover:
    def __init__(self):
        self.input = dotdict()
        self.input.V = 0.0
        self.input.omega = 0.0

        # Geometric parameters:
        self.dims = dotdict()
        # vertical offset between rover reference R to differential D
        self.dims.k1 = 0.0856       # [m]
        # forward offset between R and D
        self.dims.k2 = 0.0          # [m]
        # horizontal offset between D and wheels / l1
        self.dims.k3 = 0.3722    # [m]
        self.dims.k3bar = 0.05      # [m]
        # distance from D to steering axis of front wheels / l6
        self.dims.k4 = 0.48         # [m]
        # height of D from wheel axles / l7
        self.dims.k5 = 0.50903      # [m]
        # length of link from rocker joint to bogie joint / l3
        self.dims.k6 = 0.50         # [m]
        # length from bogie joint to forward/rear bogie / l4 / l8
        self.dims.k7f = 0.353       # [m]
        self.dims.k7r = 0.315       # [m]
        self.dims.k7 = 0.668        # [m]
        # height of bogie joint from wheel axles / l5
        self.dims.k8 = 0.424        # [m]
        # angle of link between rocker and bogie joints / l2
        self.dims.k9 = 170.0*pi/180.0  # [rad]
        self.dims.l2 = 0.1208       # [m]
        # wheel radius
        self.dims.k10 = 0.145       # [m]

        self.rot_radii = dotdict()
        self.rot_radii.r = 0.0  # rotation radius of the whole vehicle
        self.rot_radii.r1 = 0.0  # rotation radius of wheel 1
        self.rot_radii.r35 = 0.0  # rotation radius of wheels 3,5
        self.rot_radii.r2 = 0.0  # rotation radius of wheel 2
        self.rot_radii.r46 = 0.0  # rotation radius of wheels 4,6

    def calc_rotation_radii(self):
        # ICR redius calcuation
        self.rot_radii.r = self.input.omega / self.input.V
        # rotation_radii for the left side of the rover
        self.rot_radii.r1 = self.rot_radii.r - self.dims.k3
        self.rot_radii.r35 = self.rot_radii.r - self.dims.k3 - self.dims.k3bar
        # rotation_radii for the right side of the rover
        self.rot_radii.r2 = self.rot_radii.r + self.dims.k3
        self.rot_radii.r46 = self.rot_radii.r + self.dims.k3 + self.dims.k3bar

    def whl_rolling_vel(self):
        if self.input.V == 0: 
            print("V is set to 0.")
            return []
        
        self.calc_rotation_radii()
        # Velcoity calcuation for the rover
        theta_dot1 = (self.rot_radii.r1 * self.input.omega) / self.dims.k10
        theta_dot2 = (self.rot_radii.r2 * self.input.omega) / self.dims.k10
        theta_dot3 = (self.rot_radii.r35 * self.input.omega) / self.dims.k10
        theta_dot4 = (self.rot_radii.r46 * self.input.omega) / self.dims.k10
        theta_dot5 = (self.rot_radii.r35 * self.input.omega) / self.dims.k10
        theta_dot6 = (self.rot_radii.r46 * self.input.omega) / self.dims.k10

        return [theta_dot1,
                theta_dot2,
                theta_dot3,
                theta_dot4,
                theta_dot5,
                theta_dot6]


rover = DiffrentialRover()
rover.input.V = 4.0
rover.input.omega = 2.0
theta_dot = rover.whl_rolling_vel()

if len(theta_dot) != 6:
    print("Wheels rolling velocities cannot be calculated.")
    exit(1)

for i in range(len(theta_dot)):
    print('Rover Velcotiy v'+str(i) + ': ' + str(theta_dot[i]))

# mp.plot(theta_dot)
# mp.show()
