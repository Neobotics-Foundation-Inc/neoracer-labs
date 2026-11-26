"""
Neobotics Foundation Inc.
GPLv3 License
neoracer-labs/lab_g/

File Name: lab_g.py

Title: Lab G - Autonomous Parking

Author: [PLACEHOLDER] << [Write your name or team name here]

Version: 0.9.0

Purpose: This script provides the NeoRacer with the ability to autonomously detect an orange
cone and then drive and park 30cm away from the cone. Complete the lines of code under the 
#TODO indicators to complete the lab.

Expected Outcome: When the user runs the script, the NeoRacer should be fully autonomous
and drive without the assistance of the user. The NeoRacer drives according to the following
rules:
- The NeoRacer detects the orange cone using its color camera, and can navigate to the cone
and park using its color camera and LIDAR sensors.
- The NeoRacer should operate on a state machine with multiple states. There should not be
a terminal state. If there is no cone in the environment, the program should not crash.
- The NeoRacer follows the program only while the handset's mode switch is down

Environment: An orange cone on an open floor. Move the cone between runs.
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
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# TODO Part 1: Determine the HSV color threshold pairs for ORANGE
ORANGE = _____  # The HSV range for the color ORANGE

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour


########################################################################################
# Functions
########################################################################################

# [FUNCTION] Finds contours in the current color image and uses them to update 
# contour_center and contour_area
def update_contour():
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    # TODO Part 2: Complete this function by cropping the image to the bottom of the screen,
    # analyzing for contours of interest, and returning the center of the contour and the
    # area of the contour for the orange cone (Hint: Lab F)


# [FUNCTION] The start function is run once when the program starts
def start():
    global speed
    global angle

    # Initialize variables
    speed = 0
    angle = 0

    # The NeoRacer's full speed (1.0) is about 6 m/s. Uncomment the next line and pick a
    # lower limit for your room. Every speed the program sets is then scaled by it
    # rc.drive.set_max_speed(0.25)

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lab G - Autonomous Parking\n"
        "\n"
        "Controls:\n"
        "   Mode switch down = the program drives the car\n"
        "   Mode switch up = the handset drives the car\n"
        "   Speed, angle and the cone position print every half second"
    )


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until Ctrl+C is pressed
# in the terminal
def update():
    global speed
    global angle

    # Search for contours in the current color image
    update_contour()

    # TODO Part 3: Park the car 30cm away from the closest orange cone.
    # You may use a state machine and the color camera, the lidar, or both. The
    # NeoRacer has no depth camera, and rc.camera.get_depth_image() raises NotImplementedError.

    # Set the speed and angle of the NeoRacer after calculations have been complete
    rc.drive.set_speed_angle(speed, angle)


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x-position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + f" : area = {contour_area}, speed = {speed:.2f}, angle = {angle:.2f}")

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + f" : area = {contour_area}, speed = {speed:.2f}, angle = {angle:.2f}")


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
