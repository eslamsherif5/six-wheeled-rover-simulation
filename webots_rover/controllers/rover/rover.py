#!/usr/bin/python3

"""rover controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Keyboard, AnsiCodes
from rover_diffrential import *
import time

rover = DiffrentialRover()

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motors = list()
for i in range(6):
    motors.append(robot.getDevice('whl' + str(i+1) + '_drive_motor'))
    try:
        # print(motors[i].name)
        motors[i].setPosition(float('inf'))
    except:
        pass
#  ds = robot.getDevice('dsname')
kb = Keyboard()
kb.enable(timestep)

#  ds.enable(timestep)

V = 0.0
w = 0.0
rover.input.V = V       # [m/s]
rover.input.omega = w   # [rad/s]
theta_dot = rover.whl_rolling_vel() # [rad/s] 

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    k = kb.getKey()
    if k == Keyboard.UP:
        print("Increasing forward speed (forward) ... V = " + str(round(V + 0.1, 2)))    
        time.sleep(0.1)
        V = V + 0.1
        rover.input.V = V
        theta_dot = rover.whl_rolling_vel()
        for i in range(len(motors)):
            motors[i].setVelocity(theta_dot[i])
    
    if k == Keyboard.DOWN:
        print("Decreasing forward speed (reverse) ... V = " + str(round(V - 0.1, 2)))    
        time.sleep(0.1)
        V = V - 0.1
        rover.input.V = V
        theta_dot = rover.whl_rolling_vel()
        for i in range(len(motors)):
            motors[i].setVelocity(theta_dot[i])

    if k == Keyboard.LEFT:
        print("Increasing angular speed  (turning left) ... w = " + str(round(w + 0.1, 3)))    
        time.sleep(0.1)
        w = w + 0.1
        rover.input.omega = w
        theta_dot = rover.whl_rolling_vel()
        for i in range(len(motors)):
            motors[i].setVelocity(theta_dot[i])
    
    if k == Keyboard.RIGHT:
        print("Decreasing angular speed (turning right) ... w = " + str(round(w - 0.1, 3)))    
        time.sleep(0.1)
        w = w - 0.1
        rover.input.omega = w
        theta_dot = rover.whl_rolling_vel()
        for i in range(len(motors)):
            motors[i].setVelocity(theta_dot[i])
    
    if k == 83 or k == 32: # S or Spacebar => Stop the robot
        print("Stopping ... V = 0.0")    
        time.sleep(0.1)
        rover.input.V = V = 0.0
        rover.input.omega = w = 0.0
        theta_dot = rover.whl_rolling_vel()
        for i in range(len(motors)):
            motors[i].setVelocity(theta_dot[i])
        
    # if k == 87 or k == 88:     # W or X
    #     # Zero out angular speed assuming motion in a straight line
    #     rover.input.omega = 0.0
    #     # Calculate wheel rolling velocities depending only on longitudinal (forward/reverse) speed
    #     theta_dot = rover.whl_rolling_vel()
    #     for i in range(len(motors)):
    #         motors[i].setVelocity(theta_dot[i])

    # if k == 81 or k == 69 or k == 90 or k == 67:     # Q, E, Z or C
    #     # Calculate wheel rolling velocities depending on longitudinal (forward/reverse) speed and angular speed
    #     theta_dot = rover.whl_rolling_vel()
    #     for i in range(len(motors)):
    #         motors[i].setVelocity(theta_dot[i])
        
    # if k == 65 or k == 68:  # A or D => spin in place (theoritically)
    #     # Zero out longitudinal speed assuming spinning about the center of the rover
    #     rover.input.V = 0.0
    #     # Calculate wheel rolling velocities depending only on angular
    #     theta_dot = rover.whl_rolling_vel()
    #     for i in range(len(motors)):
    #         motors[i].setVelocity(theta_dot[i])
            
    if k == 72:
        print(AnsiCodes.CLEAR_SCREEN)
        
        print("Use these keys to drive the robot around.\nPress 'H' to show this help message.\n")
        
        # print("[ Q ] [ W ] [ E ]\n[ A ] [ S ] [ D ]\n[ Z ] [ X ] [ C ]\n")
        
        print("[ ↑ ]: Increase longitudinal speed (V)")
        print("[ ↓ ]: Decrease longitudinal speed (V)")
        print("[ ← ]: Increase angular speed (w)")
        print("[ → ]: Decrease angular speed (w)")
        
        print("\n[ S ] or [ Spacebar ]: Stop the robot and reset V,w to 0.")
        # print("[ X ]: Move reverse.")
        # print("[ A ]: Spin left.")
        # print("[ D ]: Spin right.")
        # print("[ Q ]: Move forward while turning left.")
        # print("[ E ]: Move forward while turning right.")
        # print("[ Z ]: Move reverse while turning left.")
        # print("[ C ]: Move reverse while turning right.")
        time.sleep(0.1)
        
    # if k != -1:
    #     print("Pressed: " + str(k))
        
    # Q = 81
    # W = 87
    # E = 69
    # A = 65
    # S = 83
    # D = 68
    # Z = 90
    # X = 88
    # C = 67
    # 
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
