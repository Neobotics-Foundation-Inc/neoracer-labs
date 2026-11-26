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

Expected Outcome: When the user runs the script and flips the handset's mode switch down,
they are able to control the NeoRacer with the sticks:
- When the throttle stick is pushed forward, the NeoRacer drives forward
- When the throttle stick is pulled back, the NeoRacer drives backward
- When the steering stick's x-axis has a value of greater than 0, the NeoRacer's wheels turn to the right
- When the steering stick's x-axis has a value of less than 0, the NeoRacer's wheels turn to the left
- Each time the mode switch is flipped down, the speed step grows by 0.25, wrapping back to 0.25
after 1.0, and the current speed step prints to the terminal window
- Finally, the speed and the angle follow how far each stick is pushed, instead of a fixed step

With the mode switch up, the handset drives the car directly and the program's drive commands
do nothing, so the program only takes over with the switch down.
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
    speed_offset = 0.25 # The initial speed step is 0.25, about 1.5 m/s on the NeoRacer
    angle_offset = 1.0 # The initial angle step is 1.0, full lock

    # The NeoRacer's full speed (1.0) is about 6 m/s. Uncomment the next line and pick a
    # lower limit for your room. Every speed the program sets is then scaled by it
    # rc.drive.set_max_speed(0.25)

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
    # throttle stick is pushed forward, the NeoRacer moves forward at the designated speed,
    # and when it is pulled back, the NeoRacer moves backward at the designated speed.
    # rc.controller.get_joystick() returns (x, y). On the NeoRacer the throttle stick is
    # Joystick.LEFT and its y value is +1 fully forward, -1 fully back and 0 at rest.
    (x, y) = rc.controller.get_joystick(rc.controller.Joystick._____)
    if y > 0.5:
        speed = _____
    elif y < -0.5:
        speed = _____
    else:
        speed = 0

    # TODO Part 2: Modify the following conditional statement such that when the
    # value of the steering stick's x-axis is greater than 0, the NeoRacer's wheels turn right.
    # When the value is less than 0, the NeoRacer's wheels turn left. On the NeoRacer the
    # steering stick is Joystick.RIGHT and its x value is +1 fully right and -1 fully left.
    (x, y) = rc.controller.get_joystick(rc.controller.Joystick._____)
    if x > 0.5:
        angle = _____
    elif x < -0.5:
        angle = -_____
    else:
        angle = 0

    # TODO Part 3: Write a conditional statement such that each time the mode switch is
    # flipped down (rc.controller.was_pressed(rc.controller.Button.RB)), speed_offset grows
    # by 0.25. When it would pass 1.0, set it back to 0.25. Print the current speed_offset
    # to the terminal window. The car only follows the program while the switch is down,
    # so flip it up, then down again, to change the step.

    # TODO Part 4: Once Parts 1 to 3 work, replace the fixed steps: set speed to
    # y * speed_offset and angle to x * angle_offset, so the car follows how far each
    # stick is pushed. Use the y from the throttle stick for the speed and the x from the
    # steering stick for the angle.

    # Send the speed and angle values to the NeoRacer
    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
