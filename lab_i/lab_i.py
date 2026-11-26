"""
Neobotics Foundation Inc.
GPLv3 License
neoracer-labs/lab_i/

File Name: lab_i.py

Title: Lab I - Wall Follower

Author: [PLACEHOLDER] << [Write your name or team name here]

Version: 0.9.0

Purpose: This script provides the NeoRacer with the ability to autonomously follow a wall.
The script should handle wall following for the right wall, the left wall, both walls, and
be flexible enough to handle very narrow and very wide walls as well.

Expected Outcome: When the user runs the script, the NeoRacer should be fully autonomous
and drive without the assistance of the user. The NeoRacer drives according to the following
rules:
- The NeoRacer detects a wall using the LIDAR sensor a certain distance and angle away.
- Ideally, the NeoRacer should be a set distance away from a wall, or if two walls are detected,
should be in the center of the walls.
- The NeoRacer may have different states depending on if it sees only a right wall, only a 
left wall, or both walls.
- Both speed and angle parameters are variable and recalculated every frame. The speed and angle
values are sent once at the end of the update() function.

Note: This file consists of bare-bones skeleton code, which is the bare minimum to run a
Python file on the NeoRacer. Less help will be provided from here on out, since you've made
it this far. Good luck, and remember to contact an instructor if you have any questions!

Environment: A course with a wall on the left, on the right, or on both sides, with narrow
and wide sections. Lowest time wins!
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


########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once when the program starts
def start():
    pass  # Remove 'pass' and write your source code for the start() function here


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until Ctrl+C is pressed
# in the terminal
def update():
    pass  # Remove 'pass' and write your source code for the update() function here


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass  # Remove 'pass' and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
