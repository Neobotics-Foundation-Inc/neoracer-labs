"""
Neobotics Foundation Inc.
GPLv3 License
neoracer-labs/

File Name: demo.py

Title: Demo NeoRacer program

Author: Neobotics

Version: 0.9.0

Purpose: To verify that the basic NeoRacer functions work and the student has set up
the system correctly to run Python scripts with the start/update paradigm on the car.

Expected Outcome: Terminal output and NeoRacer movement occur when the handset's mode
switch is flipped
- When the mode switch is flipped down, print a message and drive in an L: forward for
one second, then forward and to the right for one second, then stop
- When the mode switch is flipped up, print a message and stop
- While the mode switch is down, print a message once per second from update_slow()
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
counter = 0
isDriving = False

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once when the program starts
def start():
    # If we use a global variable in our function, we must list it at
    # the beginning of our function like this
    global counter
    global isDriving

    # The start function is a great place to give initial values to global variables
    counter = 0
    isDriving = False

    # This tells the car to begin at a standstill
    rc.drive.stop()

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until Ctrl+C is pressed
# in the terminal
def update():
    
    global counter
    global isDriving

    # On the NeoRacer, Button.RB reads as down while the handset's mode switch is down,
    # and the car follows the program only in that position. was_pressed() is True on
    # the one frame the switch goes down
    if rc.controller.was_pressed(rc.controller.Button.RB):
        print("The mode switch was flipped down")
        # Reset the counter and start driving in an L
        counter = 0
        isDriving = True # Toggle Variable

    # was_released() is True on the one frame the switch goes back up. The handset
    # drives the car again, so the program stops its own L
    if rc.controller.was_released(rc.controller.Button.RB):
        print("The mode switch was flipped up")
        isDriving = False
        rc.drive.stop()

    if isDriving:
        # rc.get_delta_time() gives the time in seconds since the last time
        # the update function was called
        counter += rc.get_delta_time()

        if counter < 1:
            # Drive forward at a quarter of full speed (about 1.5 m/s) for one second
            rc.drive.set_speed_angle(0.25, 0)
        elif counter < 2:
            # Turn right at the same speed for the next second
            rc.drive.set_speed_angle(0.25, 1)
        else:
            # Otherwise, stop the car
            rc.drive.stop()
            isDriving = False

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # is_down() is True on every frame the mode switch is down, so this prints a
    # message once per second for as long as the switch stays down
    if rc.controller.is_down(rc.controller.Button.RB):
        print("The mode switch is down (update_slow)")


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
