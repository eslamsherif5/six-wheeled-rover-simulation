#!/usr/bin/python3

"""rover controller."""

from controller import Robot, Keyboard, AnsiCodes
from rover_diffrential import *
import time

def print_help():
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

rover = DiffrentialRover()

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motors = list()
for i in range(6):
    motors.append(robot.getDevice('whl' + str(i+1) + '_drive_motor'))
    try:
        # print(motors[i].name)
        motors[i].setPosition(float('inf'))
    except:
        pass

kb = Keyboard()
kb.enable(timestep)

V = 0.0
w = 0.0
rover.input.V = V       # [m/s]
rover.input.omega = w   # [rad/s]
theta_dot = rover.whl_rolling_vel()  # [rad/s]
for i in range(len(motors)):
    motors[i].setVelocity(theta_dot[i])
    
print_help()
print("\n\n\n")

while robot.step(timestep) != -1:
    k = kb.getKey()
    if k == Keyboard.UP:
        print("[ ↑ ] Moving forward ... V = " +
              str(round(V + 0.1, 2)))
        # wait for a very little while
        time.sleep(0.2)
        # increase the speed
        V = V + 0.1
        rover.input.V = V
        # apply the new speed
        theta_dot = rover.whl_rolling_vel()
        for i in range(len(motors)):
            motors[i].setVelocity(theta_dot[i])

    if k == Keyboard.DOWN:
        print("[ ↓ ] Moving reverse ... V = " +
              str(round(V - 0.1, 2)))
        # wait for a very little while
        time.sleep(0.2)
        # decrease the speed
        V = V - 0.1
        rover.input.V = V
        # apply the new speed
        theta_dot = rover.whl_rolling_vel()
        for i in range(len(motors)):
            motors[i].setVelocity(theta_dot[i])

    if k == Keyboard.LEFT:
        print("[ ← ] Turning left ... w = " +
              str(round(w + 0.1, 3)))
        # wait for a very little while
        time.sleep(0.2)
        # increase the speed
        w = w + 0.1
        rover.input.omega = w
        # apply the new speed
        theta_dot = rover.whl_rolling_vel()
        for i in range(len(motors)):
            motors[i].setVelocity(theta_dot[i])

    if k == Keyboard.RIGHT:
        print("[ → ] Turning right ... w = " +
              str(round(w - 0.1, 3)))
        # wait for a very little while
        time.sleep(0.2)
        # decrease the speed
        w = w - 0.1
        rover.input.omega = w
        # apply the new speed
        theta_dot = rover.whl_rolling_vel()
        for i in range(len(motors)):
            motors[i].setVelocity(theta_dot[i])

    if k == 83 or k == 32:  # S or Spacebar => Stop the robot
        print("[ S ] Stopping ... V = 0.0")
        time.sleep(0.1)
        rover.input.V = V = 0.0
        rover.input.omega = w = 0.0
        theta_dot = rover.whl_rolling_vel()
        for i in range(len(motors)):
            motors[i].setVelocity(theta_dot[i])

    if k == 72:     # H
        print_help()
    
    # if k != -1:   # nothing is pressed
    #     print("Pressed: " + str(k))   # print the equivalent ascii code

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

# Enter here exit cleanup code.
