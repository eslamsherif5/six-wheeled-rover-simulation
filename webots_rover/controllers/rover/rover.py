#!/usr/bin/python3

"""rover controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Keyboard, AnsiCodes

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
        print(motors[i].name)
        motors[i].setPosition(float('inf'))
    except:
        pass
#  ds = robot.getDevice('dsname')
kb = Keyboard()
kb.enable(timestep)

#  ds.enable(timestep)

1.7623
12.03
1.073
12.72
1.073
12.72

v = [1.7623, 
     12.03, 
     1.073, 
     12.72, 
     1.073, 
     12.72]

# v = [-2.0, 
#      2.0, 
#      -2.0, 
#      2.0, 
#      -2.0, 
#      2.0]

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    k = kb.getKey()
    if k == Keyboard.UP:
        print("Forward.")    
        for i in range(len(motors)):
            motors[i].setVelocity(v[i])
    
    if k == Keyboard.DOWN:
        print("Forward.")    
        for i in range(len(motors)):
            motors[i].setVelocity(-v[i])
    
    if k == 32 or k == -1: # spacebar
        for i in range(len(motors)):
            motors[i].setVelocity(0.0)
    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
