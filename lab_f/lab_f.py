"""
Neobotics Foundation Inc.
GPLv3 License
neoracer-labs/lab_f/

File Name: lab_f.py

Title: Lab F - Line Follower

Author: [PLACEHOLDER] << [Write your name or team name here]

Version: 0.9.0

Purpose: Write a script to enable fully autonomous behavior from the NeoRacer. The
NeoRacer should automatically identify the color of a line it sees, then drive on the
center of the line throughout the obstacle course. The NeoRacer should also identify
color changes, following colors with higher priority than others. Complete the lines 
of code under the #TODO indicators to complete the lab.

Expected Outcome: When the user runs the script and flips the handset's mode switch down,
they are able to control the NeoRacer's speed while the program steers:
- When the throttle stick is pushed forward, the NeoRacer moves forward
- When the throttle stick is pulled back, the NeoRacer moves backward
- The angle of the NeoRacer should only be controlled by the center of the line contour
- The NeoRacer sees the color RED as the highest priority, then GREEN, then BLUE
"""

########################################################################################
# Imports
########################################################################################

import cv2 as cv
import numpy as np

import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car
CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

# TODO Part 1: Determine the HSV color threshold pairs for GREEN and RED
# Colors, stored as a pair (hsv_min, hsv_max) Hint: Lab E!
BLUE = ((90, 50, 50), (120, 255, 255))  # The HSV range for the color blue
GREEN = _____  # The HSV range for the color green
RED = _____  # The HSV range for the color red

# Color priority: Red >> Green >> Blue
COLOR_PRIORITY = (RED, GREEN, BLUE)

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

    # On the NeoRacer the first call subscribes to the camera and can wait up to a second
    # for the first frame. Later calls return the newest frame at once
    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        image = rc_utils.crop(image, _____, _____)

        # TODO Part 2: Search for line colors, and update the global variables
        # contour_center and contour_area with the largest contour found

        # Display the image to the screen
        rc.display.show_color_image(image)

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
        ">> Lab F - Line Follower\n"
        "\n"
        "Controls:\n"
        "   Mode switch down = the program drives the car\n"
        "   Throttle stick = drive forward or backward while the program steers\n"
        "   Mode switch up = the handset drives the car\n"
        "   Speed, angle and the line position print every half second"
    )

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until Ctrl+C is pressed
# in the terminal
def update():
    """
    After start() is run, this function is run every frame until Ctrl+C is pressed
    in the terminal
    """
    global speed
    global angle

    # Search for contours in the current color image
    update_contour()

    # TODO Part 3: Determine the angle that the NeoRacer should receive based on the current 
    # position of the center of line contour on the screen. Hint: The NeoRacer should drive in
    # a direction that moves the line back to the center of the screen.

    # Choose an angle based on contour_center
    # If we could not find a contour, keep the previous angle
    if contour_center is not None:
        angle = _____

    # Use the throttle stick to control the car's speed. On the NeoRacer the throttle stick
    # is Joystick.LEFT: its y value is +1 fully forward, -1 fully back and 0 at rest, so
    # the car stops when the stick is released. 1.0 is full speed, about 6 m/s, unless
    # start() sets a lower max speed
    _, speed = rc.controller.get_joystick(rc.controller.Joystick.LEFT)

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
