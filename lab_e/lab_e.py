"""
Neobotics Foundation Inc.
GPLv3 License
neoracer-labs/lab_e/

File Name: lab_e.py

Title: Lab E - Stoplight Challenge

Author: [PLACEHOLDER] << [Write your name or team name here]

Version: 0.9.0

Purpose: Write a script to enable autonomous behavior from the NeoRacer. When
the NeoRacer sees a stoplight, respond accordingly
by going straight, turning right, turning left, or stopping. Append instructions to the
queue depending on whether the position of the NeoRacer relative to the stoplight reaches
a certain threshold, and be able to respond to traffic lights at consecutive intersections. 

Expected Outcome: When the user runs the script, the NeoRacer should control itself using
the following constraints:
- When the NeoRacer sees a BLUE traffic light, make a right turn at the intersection
- When the NeoRacer sees an ORANGE traffic light, make a left turn at the intersection
- When the NeoRacer sees a GREEN traffic light, go straight
- When the NeoRacer sees a RED traffic light, stop moving,
- When the NeoRacer sees any other traffic light colors, stop moving.
- The NeoRacer follows the program only while the handset's mode switch is down. Flipping
it up hands the car back to the handset and clears the queue

Considerations: Since the user is not controlling the NeoRacer, be sure to consider the
following scenarios:
- What should the NeoRacer do if it sees two traffic lights, one at the current intersection
and the other at the intersection behind it?
- What should be the constraint for adding the instructions to the queue? Traffic light position,
traffic light area, or both?
- How often should the instruction-adding function calls be? Once, twice, or 60 times a second?

Environment: A course with intersections and a stoplight prop at each one. The prop shows
one color at a time. Change its color to test each case.
"""

########################################################################################
# Imports
########################################################################################

import cv2 as cv
import numpy as np

import racecar_core

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour (Adjust threshold!)
MIN_CONTOUR_AREA = _____

# TODO Part 1: Determine the HSV color threshold pairs for ORANGE, GREEN, RED, YELLOW, and PURPLE
# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 50, 50), (120, 255, 255))  # The HSV range for the color blue
GREEN = _____  # The HSV range for the color green
RED = _____  # The HSV range for the color red
ORANGE = _____ # The HSV range for the color orange
YELLOW = _____ # The HSV range for the color yellow
PURPLE = _____ # The HSV range for the color purple

# >> Variables
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

queue = [] # The queue of instructions
stoplight_color = "" # The current color of the stoplight
speed = 0.0 # The current speed of the car
angle = 0.0 # The current angle of the car's wheels

########################################################################################
# Functions
########################################################################################

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour():
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found

        # TODO Part 3: Repeat the search for all potential traffic light colors,
        # then select the correct color of traffic light detected.

        # Display the image to the screen
        rc.display.show_color_image(image)

# [FUNCTION] The start function is run once when the program starts
def start():

    # The NeoRacer's full speed (1.0) is about 6 m/s. Uncomment the next line and pick a
    # lower limit for your room. Every speed the program sets is then scaled by it
    # rc.drive.set_max_speed(0.25)

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(0,0)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message (You may edit this to be more informative!)
    print(
        ">> Lab E - Stoplight Challenge\n"
        "\n"
        "Controls:\n"
        "   Mode switch down = the program drives the car\n"
        "   Mode switch up = the handset drives the car and the queue is cleared\n"
        "   Speed, angle, stoplight color and contour print every half second"
    )

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until Ctrl+C is pressed
# in the terminal
def update():
    global queue
    global speed
    global angle

    update_contour()

    # TODO Part 2: Complete the conditional tree with the given constraints.
    if stoplight_color == "_____":
        # Call the correct function to append the instructions to the list
    elif stoplight_color == "_____":
        # Call the correct function to append the instructions to the list
    
    # ... You may need more elif/else statements

    # TODO Part 3: Implement a way to execute instructions from the queue once they have been placed
    # by the traffic light detector logic (Hint: Lab D)

    # Send speed and angle commands to the NeoRacer
    rc.drive.set_speed_angle(speed, angle)

    # When the mode switch is flipped up, the handset drives the car. Clear the queue so
    # the program starts fresh when the switch comes back down
    if rc.controller.was_released(rc.controller.Button.RB):
        stopNow()

# [FUNCTION] update_slow() is called every half second (see start()). It prints the
# values a student needs while tuning, in place of printing every frame
def update_slow():
    print("Speed:", speed, "Angle:", angle, "Stoplight:", stoplight_color)
    if contour_center is None:
        print("No contour found")
    else:
        print("Center:", contour_center, "Area:", contour_area)

# [FUNCTION] Appends the correct instructions to make a 90 degree right turn to the queue
def turnRight():
    global queue

    # TODO Part 4: Complete the rest of this function with the instructions to make a right turn

# [FUNCTION] Appends the correct instructions to make a 90 degree left turn to the queue
def turnLeft():
    global queue

    # TODO Part 5: Complete the rest of this function with the instructions to make a left turn

# [FUNCTION] Appends the correct instructions to go straight through the intersection to the queue
def goStraight():
    global queue

    # TODO Part 6: Complete the rest of this function with the instructions to go straight

# [FUNCTION] Clears the queue to stop all actions
def stopNow():
    global queue
    queue.clear()

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()