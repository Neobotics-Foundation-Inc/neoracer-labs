"""
Neobotics Foundation Inc.
GPLv3 License
neoracer-labs/lab_b/

File Name: lab_b.py

Title: Lab B - Printing Statements Using Controller

Author: [PLACEHOLDER] << [Write your name or team name here]

Version: 0.9.0

Purpose: Print several statements to the terminal window from the update() function
and gain an understanding of how to poll data from the controller. Complete the lines of
code under the #TODO indicators to complete the lab.

Expected Outcome: At the end of the lab, running the script will print several
statements to the terminal window based on handset input. On the NeoRacer the handset's
mode switch is the button: Button.RB reads as down while the switch is down.
- When the mode switch is flipped down, print "Hello World!" to the terminal window once
- When the mode switch is flipped back up, print "Welcome to the NeoRacer, <your name>!"
once
- While the mode switch is down, print the current elapsed time of the script to the
terminal window in seconds, every frame, for as long as the switch stays down. Round the
time to 2 decimal places.
- When the throttle stick is pushed forward, print the current elapsed time of the script
to the terminal window in seconds. The script should only print the time once after the
stick is pushed, however long it is held, until it comes back to the center. Round the
time to 2 decimal places.
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
global your_name
global counter
counter = 0

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once when the program starts
def start():
    global your_name

    # TODO Part 1: Modify the variable below to your name. Modify the print statement,
    # such that when run, the program prints "Hello {your name}, welcome to the NeoRacer!"
    your_name = "_____"
    # print(f"Hello {_____}, welcome to the NeoRacer!")

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until Ctrl+C is pressed
# in the terminal
def update():
    global counter
    counter += rc.get_delta_time()

    # TODO Part 2: Modify the following if statement's contents to print out "Hello World!"
    # to the terminal window after the mode switch has been flipped down. was_pressed()
    # is True on the one frame the switch goes down.
    if rc.controller.was_pressed(rc.controller.Button.RB):
        print("_____")

    # TODO Part 3: Modify the following if statement to print out "Welcome to the NeoRacer,
    # <your name>!" after the mode switch has been flipped back up. Which controller
    # function is True on the one frame the switch goes up?
    if rc.controller._____(rc.controller.Button._):
        print(f"Welcome to the NeoRacer, {_____}!")

    # TODO Part 4: Modify the following if statement to print out the current elapsed
    # time of the script to the terminal window in seconds while the mode switch is down.
    # The script should keep printing time updates for as long as the switch stays down.
    # Round the time to 2 decimal places.
    if rc.controller._____(rc.controller.Button._):
        print(f"The current script has been running for {_____} seconds!")

    # TODO Part 5: Create an if statement below to print out the current elapsed time of
    # the script to the terminal window in seconds when the throttle stick is pushed
    # forward. rc.controller.get_joystick(rc.controller.Joystick.LEFT) returns (x, y),
    # and y is above 0.5 while the stick is pushed forward. The script should only print
    # the time once each time the stick is pushed, however long it is held. (Hint: a
    # global variable can remember whether the stick was already forward on the last frame.)
    # Round the time to 2 decimal places.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
