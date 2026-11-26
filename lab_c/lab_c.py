"""
Neobotics Foundation Inc.
GPLv3 License
neoracer-labs/lab_c/

File Name: lab_c.py

Title: Lab C - NeoRacer Controller

Author: [PLACEHOLDER] << [Write your name or team name here]

Version: 0.9.0

Purpose: Using a Python script and the data polled from the controller module,
write code to replicate a manual control scheme for the NeoRacer. Gain a mastery
in using conditional statements, controller functions and an understanding in the
rc.drive.set_speed_angle() function. Complete the lines of code under the #TODO indicators 
to complete the lab.

Expected Outcome: When the user runs the script, they are able to control the NeoRacer
using the following keys:
- When the right trigger is pressed, the NeoRacer drives forward
- When the left trigger is pressed, the NeoRacer drives backward
- When the left joystick's x-axis has a value of greater than 0, the NeoRacer's wheels turns to the right
- When the left joystick's x-axis has a value of less than 0, the NeoRacer's wheels turns to the left
- When the "A" button is pressed, increase the speed and print the current speed to the terminal window
- When the "B" button is pressed, reduce the speed and print the current speed to the terminal window
- When the "X" button is pressed, increase the turning angle and print the current turning angle to the terminal window
- When the "Y" button is pressed, reduce the turning angle and print the current turning angle to the terminal window
"""

########################################################################################
# Imports
########################################################################################

import racecar_core

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
global speed
global angle
global speed_offset
global angle_offset

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once when the program starts
def start():
    global speed
    global angle
    global speed_offset
    global angle_offset

    speed = 0.0 # The initial speed is 0.0
    angle = 0.0 # The initial turning angle away from the center is at 0.0
    speed_offset = 0.5 # The initial speed offset is 0.5
    angle_offset = 1.0 # The initial angle offset is 1.0

    # This tells the car to begin at a standstill
    rc.drive.stop()

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until Ctrl+C is pressed
# in the terminal
def update():
    global speed
    global angle
    global speed_offset
    global angle_offset
    
    # TODO Part 1: Modify the following conditional statement such that when the
    # right trigger is pressed, the NeoRacer moves forward at the designated speed.
    # when the left trigger is pressed, the NeoRacer moves backward at the designated speed.
    if rc.controller.get_trigger(rc.controller.Trigger._____) > _____:
        speed = _____
    elif rc.controller.get_trigger(rc.controller.Trigger._____) > _____:
        speed = _____
    else:
        speed = 0
      
    # TODO Part 2: Modify the following conditional statement such that when the
    # value of the left joystick's x-axis is greater than 0, the NeoRacer's wheels turn right.
    # When the value of the left joystick's x-axis is less than 0, the NeoRacer's wheels turn left.
    (x, y) = rc.controller.get_joystick(rc.controller.Joystick._____)
    if x > 0.5:
        angle = _____
    elif x < -0.5:
        angle = -_____
    else:
        angle = 0

    # TODO Part 3: Write a conditional statement such that when the
    # "A" button is pressed, increase the speed of the NeoRacer. When the "B" button is pressed,
    # decrease the speed of the NeoRacer. Print the current speed of the NeoRacer to the
    # terminal window.

    # TODO Part 4: Write a conditional statement such that when the
    # "X" button is pressed, increase the turning angle of the NeoRacer. When the "Y" button 
    # is pressed, decrease the turning angle of the NeoRacer. Print the current turning angle 
    # of the NeoRacer to the terminal window.

    # Send the speed and angle values to the NeoRacer
    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
