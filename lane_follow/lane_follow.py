"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs

File Name: lane_follow.py

Title: Lane Following - Two Blue Lines

Author: Neobotics

Purpose: Drive the NeoRacer down a lane marked by two lines of blue tape and stay as
close to the middle of the lane as possible. Each frame, the camera image is cropped to
the strip of floor in front of the car, the biggest blue blob on each side of the image
is taken as that side's lane line, and the midpoint between the two lines is where the
car wants to be. A PID controller turns the distance between that midpoint and the centre
of the image into a steering angle. Speed is constant for now.

Expected Outcome: When the user runs the script, the RACECAR drives forward at a fixed
speed and steers to keep itself centred between the two blue lines. A window on the
Jetson desktop shows the live camera image with the search band, the detected lines, the
lane centre and the steering command drawn on top.
- Both lines visible: steer toward the midpoint between them
- One line visible: steer toward a point half a lane width inside it
- No line visible: hold the last steering angle
- Ctrl+C stops the program; the driver zeroes the motor half a second later
"""

########################################################################################
# Imports
########################################################################################

import cv2 as cv

import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The HSV colour range for blue tape, as (hsv_min, hsv_max). OpenCV hue runs 0..179 and
# blue sits near 100..115. If the mask misses the tape, widen the range; if it grabs the
# floor, tighten it. utility/hsv-p_tuner.py or the linefollow dashboard (port 8086) show
# the mask live while you drag sliders.
BLUE = ((90, 50, 50), (120, 255, 255))

# The smallest blob (in pixels) we count as a piece of tape. Raise it to ignore specks.
MIN_CONTOUR_AREA = 200

# The search band: the strip of floor directly in front of the car, given as a fraction
# of the image height measured from the top. 0.6 means "the bottom 40% of the image".
# Lower the number to look farther ahead, raise it to look closer to the bumper.
CROP_TOP_FRAC = 0.6

# How far apart the two lines are in pixels, used to aim when only one line is visible.
# This is only a starting guess: it is replaced by the measured width as soon as both
# lines are seen together.
LANE_WIDTH_GUESS = 400

# How fast to drive. Constant for now. The number is a fraction of full scale, and full
# scale is the library's max speed (1.0) x the driver's throttle cap (max_speed_forward
# in throttle.yaml) x the firmware's 6 m/s ceiling, so with the shipped settings 0.2 is
# roughly 1.2 m/s. Start low and raise it once the steering is tuned.
SPEED = 0.2

# PID gains. The error is the lane centre's offset from the image centre, normalised to
# -1 (lane centre at the left edge of the image) .. +1 (right edge). KP = 1.0 therefore
# asks for full lock when the lane centre is at the edge and half lock half way there.
KP = 1.0    # proportional: steer in proportion to how far off centre the car is
KI = 0.0    # integral: slowly removes a steady offset. Leave at 0 until KP and KD feel right
KD = 0.02   # derivative: damps the swing when the error is changing quickly
INTEGRAL_LIMIT = 0.5  # the integral term can never ask for more than this much steering

# >> Variables
speed = 0.0          # The current speed of the car
angle = 0.0          # The current angle of the car's wheels
error = 0.0          # Lane centre offset, -1..1. Positive = lane centre is to the right
last_error = 0.0     # The error from the previous frame, for the derivative term
integral = 0.0       # The running sum of error over time, for the integral term
lane_width = LANE_WIDTH_GUESS  # Pixels between the two lines the last time both were seen
left_center = None   # The (pixel row, pixel column) of the left line in the full image
right_center = None  # The (pixel row, pixel column) of the right line in the full image
lane_center_col = None  # The pixel column of the middle of the lane, or None if unseen
image_width = 640    # The width of the last camera frame, for the update_slow print-out


########################################################################################
# Functions
########################################################################################

# [FUNCTION] Converts a contour found in the cropped band into its centre in full-image
# coordinates, so it can be drawn on and compared with the whole picture. Returns None
# when there is no contour.
def line_center(contour, crop_top):
    if contour is None:
        return None
    center = rc_utils.get_contour_center(contour)
    if center is None:
        return None
    # The band starts crop_top rows down the full image, so shift the row back by that
    return (center[0] + crop_top, center[1])


# [FUNCTION] Finds the two blue tape lines in the floor band of the current colour image
# and updates left_center, right_center and lane_width. Returns the image with the
# detections drawn on it, or None if the camera has not delivered a frame yet.
def update_lines():
    global left_center, right_center, lane_width, image_width

    image = rc.camera.get_color_image()

    # The camera node may not have published yet on the first few frames after start()
    if image is None:
        left_center = None
        right_center = None
        return None

    # Use the real frame size rather than rc.camera.get_width() / get_height(): those
    # return the 640x480 constants and do not follow a changed camera resolution
    height, width = image.shape[0], image.shape[1]
    image_width = width
    crop_top = int(height * CROP_TOP_FRAC)

    # Crop to the floor directly in front of the car. crop() returns a view into image,
    # so anything we draw on the band also appears in the full image we display later
    band = rc_utils.crop(image, (crop_top, 0), (height, width))

    # Find every blue blob in the band, then sort the blobs into the left half and the
    # right half of the image by where their centre is
    contours = rc_utils.find_contours(band, BLUE[0], BLUE[1])
    left_contours = []
    right_contours = []
    for contour in contours:
        center = rc_utils.get_contour_center(contour)
        if center is None:
            continue
        if center[1] < width // 2:
            left_contours.append(contour)
        else:
            right_contours.append(contour)

    # The biggest blob on each side is that side's tape line. Anything smaller than
    # MIN_CONTOUR_AREA is ignored, so get_largest_contour returns None for an empty side
    left_line = rc_utils.get_largest_contour(left_contours, MIN_CONTOUR_AREA)
    right_line = rc_utils.get_largest_contour(right_contours, MIN_CONTOUR_AREA)

    left_center = line_center(left_line, crop_top)
    right_center = line_center(right_line, crop_top)

    # Whenever both lines are in view, remember how far apart they are. That distance
    # is what lets us keep aiming for the middle when one line drops out of view
    if left_center is not None and right_center is not None:
        lane_width = right_center[1] - left_center[1]

    # Draw everything blue in grey, then the two chosen lines in green, so the window
    # shows both what the threshold picked up and what the controller is using
    for contour in contours:
        rc_utils.draw_contour(band, contour, rc_utils.ColorBGR.dark_gray.value)
    if left_line is not None:
        rc_utils.draw_contour(band, left_line, rc_utils.ColorBGR.green.value)
        rc_utils.draw_circle(image, left_center, rc_utils.ColorBGR.yellow.value)
    if right_line is not None:
        rc_utils.draw_contour(band, right_line, rc_utils.ColorBGR.green.value)
        rc_utils.draw_circle(image, right_center, rc_utils.ColorBGR.yellow.value)

    return image


# [FUNCTION] Draws the search band, the image centre, the lane centre and the current
# command on the image so the display window explains what the car is doing
def draw_overlay(image):
    height, width = image.shape[0], image.shape[1]
    crop_top = int(height * CROP_TOP_FRAC)
    mid_row = (crop_top + height) // 2

    # The band we search (gold box) and the centre of the image (grey line)
    cv.rectangle(image, (0, crop_top), (width - 1, height - 1), (0, 215, 255), 1)
    cv.line(image, (width // 2, crop_top), (width // 2, height), (200, 200, 200), 1)

    # Where the lane centre is (red line) and the gap the PID is closing (red bar)
    if lane_center_col is not None:
        cv.line(image, (lane_center_col, crop_top), (lane_center_col, height), (0, 0, 255), 2)
        cv.line(image, (width // 2, mid_row), (lane_center_col, mid_row), (0, 0, 255), 2)

    # The numbers, top left
    text = f"error {error:+.2f}   angle {angle:+.2f}   speed {speed:.2f}"
    cv.putText(image, text, (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed, angle, error, last_error, integral, lane_width

    # Initialize variables
    speed = 0.0
    angle = 0.0
    error = 0.0
    last_error = 0.0
    integral = 0.0
    lane_width = LANE_WIDTH_GUESS

    # Begin at a standstill
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lane Following - Two Blue Lines\n"
        "\n"
        f"Speed is fixed at {SPEED}. Steering comes from a PID on the lane centre\n"
        f"(KP = {KP}, KI = {KI}, KD = {KD}).\n"
        "A window shows the camera with the search band, the lines and the lane centre.\n"
        "Press Ctrl+C to stop."
    )


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed
def update():
    global speed, angle, error, last_error, integral, lane_center_col

    # Search for the two lines in the current colour image
    image = update_lines()

    # Work out which pixel column is the middle of the lane
    if left_center is not None and right_center is not None:
        # Both lines: the middle is half way between them
        lane_center_col = (left_center[1] + right_center[1]) // 2
    elif left_center is not None:
        # Only the left line: the middle is half a lane width to its right
        lane_center_col = left_center[1] + lane_width // 2
    elif right_center is not None:
        # Only the right line: the middle is half a lane width to its left
        lane_center_col = right_center[1] - lane_width // 2
    else:
        # No lines: we do not know where the lane is this frame
        lane_center_col = None

    # PID controller. If we can see the lane, turn its offset into a steering angle.
    # If we cannot, keep the previous angle so a short gap does not straighten the wheels
    if lane_center_col is not None and image is not None:
        width = image.shape[1]

        # Error: how far the lane centre is from the image centre, as a fraction of half
        # the image width. Positive means the lane is to the right of the car, and the
        # car must turn right (positive angle) to get back to the middle
        error = (lane_center_col - width / 2) / (width / 2)

        # Integral: the error added up over time, so a small steady offset eventually
        # adds up to a correction. Clamped so it cannot wind up and saturate the steering
        dt = rc.get_delta_time()
        integral = rc_utils.clamp(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)

        # Derivative: how fast the error is changing, which resists overshoot
        derivative = (error - last_error) / dt if dt > 0 else 0.0
        last_error = error

        angle = rc_utils.clamp(KP * error + KI * integral + KD * derivative, -1.0, 1.0)

    # Constant speed for now
    speed = SPEED

    # Send the speed and angle to the RACECAR
    rc.drive.set_speed_angle(speed, angle)

    # Show the camera image with everything drawn on it
    if image is not None:
        draw_overlay(image)
        rc.display.show_color_image(image)


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    # Print a line of ascii text showing the two lines (L, R), the lane centre (|) and
    # the numbers the controller is using
    if rc.camera.get_color_image() is None:
        print("X" * 10 + " (No image) " + "X" * 10)
        return

    s = ["-"] * 32
    bucket = max(1, image_width // 32)
    if left_center is not None:
        s[min(31, left_center[1] // bucket)] = "L"
    if right_center is not None:
        s[min(31, right_center[1] // bucket)] = "R"
    if lane_center_col is not None:
        s[min(31, max(0, lane_center_col // bucket))] = "|"

    print(
        "".join(s)
        + f" : error = {error:+.2f}  angle = {angle:+.2f}  lane width = {lane_width} px"
    )


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
