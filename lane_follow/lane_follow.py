"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs

File Name: lane_follow.py

Title: Lane Following - Two Blue Lines

Author: Neobotics

Purpose: Drive the NeoRacer down a lane marked by two lines of blue tape and stay as
close to the middle of the lane as possible, through corners as well as straights. The
floor in front of the car is cut into several horizontal strips, from the bumper out to
the far end of the view. In every strip the two tape lines are found by looking near
where they were a frame ago, and the middle of the lane is marked. The nearest strip
says how far off centre the car is right now. The farthest strip says where the lane is
heading, so the car starts turning before it reaches the corner. When a line leaves the
picture in a strip, its position is predicted from the other line and the lane width
that strip measured earlier, so one missing line no longer pulls the car off centre.
Speed is constant for now.

Expected Outcome: When the user runs the script, the RACECAR drives forward at a fixed
speed and steers to keep itself centred between the two blue lines. A window on the
Jetson desktop shows the live camera image with the strips, the detected and predicted
line positions, the lane centre path and the steering command drawn on top.
- Straight lane: drives down the middle
- Corner ahead: the far strips see the lane bend and the car begins to turn early
- One line out of view: its position is predicted from the other line, the car stays centred
- No line anywhere: hold the last steering angle
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

# The smallest blob (in pixels, inside one strip) we count as a piece of tape
MIN_CONTOUR_AREA = 60

# How much of the image we search, as a fraction of the height measured from the top.
# 0.45 means "the bottom 55% of the image". Lower it to look farther ahead (earlier
# warning of corners, but more floor clutter), raise it to look closer to the bumper.
CROP_TOP_FRAC = 0.45

# The searched area is cut into this many horizontal strips. Strip 0 is nearest the car
# and gives the lateral position; the last strip is farthest and gives the heading.
NUM_STRIPS = 4

# A tape line may not move more than this many pixels between one frame and the next
# (or between one strip and the strip below it). Blobs farther away than this from
# where the line is expected are ignored as not being that line.
MAX_JUMP = 150

# Two blobs closer together than this cannot be the two lane lines: they are two pieces
# of the same line (a gap in the tape, a glare spot), so only one of them is kept.
MIN_LANE_WIDTH = 100

# How far apart the two lines are in pixels before the car has measured it for itself.
# Each strip replaces this with its own measurement the first time it sees both lines
# (the far strips measure smaller widths because of perspective).
LANE_WIDTH_GUESS = 400

# How fast to drive. Constant for now. The number is a fraction of full scale, and full
# scale is the library's max speed (1.0) x the driver's throttle cap (max_speed_forward
# in throttle.yaml) x the firmware's 6 m/s ceiling, so with the shipped settings 0.2 is
# roughly 1.2 m/s. Start low and raise it once the steering is tuned.
SPEED = 0.2

# Steering gains. Both inputs are normalised to -1..1 across half the image width.
#   offset:  where the lane centre is in the nearest strip, relative to the image centre.
#            Positive = lane is to the right of the car.
#   heading: how far the lane centre moves sideways between the nearest strip and the
#            farthest one. Positive = the lane bends to the right up ahead.
# The PID on offset keeps the car centred. K_HEADING adds steering for the bend that is
# coming, before the offset has had a chance to grow: that is the prediction.
KP = 1.0           # proportional: steer in proportion to how far off centre the car is
KI = 0.0           # integral: slowly removes a steady offset. Leave at 0 until KP, KD feel right
KD = 0.02          # derivative: damps the swing when the offset is changing quickly
K_HEADING = 0.6    # feed-forward: how hard to pre-steer for the bend the far strips see
INTEGRAL_LIMIT = 0.5  # the integral term can never ask for more than this much steering

# >> Variables
speed = 0.0        # The current speed of the car
angle = 0.0        # The current angle of the car's wheels
offset = 0.0       # Lane centre offset in the nearest strip, -1..1
heading = 0.0      # Sideways drift of the lane centre from near strip to far strip, -1..1
last_offset = 0.0  # The offset from the previous frame, for the derivative term
integral = 0.0     # The running sum of offset over time, for the integral term
image_width = 640  # The width of the last camera frame, for the update_slow print-out

# Per-strip memory, index 0 = nearest the car. The columns are where each line is
# (seen, or predicted when out of view) and double as "where to look next frame".
strip_left = [None] * NUM_STRIPS     # Pixel column of the left line in each strip
strip_right = [None] * NUM_STRIPS    # Pixel column of the right line in each strip
strip_center = [None] * NUM_STRIPS   # Pixel column of the lane centre in each strip
strip_width = [LANE_WIDTH_GUESS] * NUM_STRIPS  # Lane width each strip last measured
left_seen = [False] * NUM_STRIPS     # True where the left line was actually in view
right_seen = [False] * NUM_STRIPS    # True where the right line was actually in view
near_center_col = None   # Lane centre in the nearest strip that found one
far_center_col = None    # Lane centre in the farthest strip that found one


########################################################################################
# Functions
########################################################################################

# [FUNCTION] Where to expect a line in strip i: where it was last frame, otherwise where
# it is in the strip just below (already found this frame), otherwise the fallback
def expected_column(memory, i, fallback):
    if memory[i] is not None:
        return memory[i]
    if i > 0 and memory[i - 1] is not None:
        return memory[i - 1]
    return fallback


# [FUNCTION] The blob column nearest to where we expect a line, or None if the nearest
# one is more than MAX_JUMP pixels away (then it is not this line)
def nearest_column(columns, expected):
    best = None
    for col in columns:
        if best is None or abs(col - expected) < abs(best - expected):
            best = col
    if best is None or abs(best - expected) > MAX_JUMP:
        return None
    return best


# [FUNCTION] Finds the two tape lines in every strip of the current colour image and
# fills in the per-strip memory, near_center_col and far_center_col. Returns the image
# with the detections drawn on it, or None if the camera has not delivered a frame yet.
def update_lines():
    global image_width, near_center_col, far_center_col

    image = rc.camera.get_color_image()

    # The camera node may not have published yet on the first few frames after start()
    if image is None:
        for i in range(NUM_STRIPS):
            strip_center[i] = None
        near_center_col = None
        far_center_col = None
        return None

    # Use the real frame size rather than rc.camera.get_width() / get_height(): those
    # return the 640x480 constants and do not follow a changed camera resolution
    height, width = image.shape[0], image.shape[1]
    image_width = width
    band_top = int(height * CROP_TOP_FRAC)
    strip_height = (height - band_top) // NUM_STRIPS

    for i in range(NUM_STRIPS):
        # Strip 0 sits at the bottom of the image (nearest the bumper), the last strip at
        # the top of the searched band (farthest away). crop() returns a view into image,
        # so anything drawn on the strip also appears in the full image we display later
        bottom = height - i * strip_height
        top = bottom - strip_height
        strip = rc_utils.crop(image, (top, 0), (bottom, width))

        # Where each line should be: last frame's position, else the strip below, else a
        # lane width either side of the image centre
        expected_left = expected_column(strip_left, i, width // 2 - strip_width[i] // 2)
        expected_right = expected_column(strip_right, i, width // 2 + strip_width[i] // 2)

        # Every blue blob in this strip that is big enough to be tape, as a column.
        # Strips span the full image width, so a strip column is also an image column
        contours = rc_utils.find_contours(strip, BLUE[0], BLUE[1])
        columns = []
        for contour in contours:
            if rc_utils.get_contour_area(contour) < MIN_CONTOUR_AREA:
                continue
            center = rc_utils.get_contour_center(contour)
            if center is None:
                continue
            columns.append(center[1])
            rc_utils.draw_contour(strip, contour, rc_utils.ColorBGR.dark_gray.value)

        columns.sort()
        if len(columns) == 2 and columns[1] - columns[0] >= MIN_LANE_WIDTH:
            # Exactly two blobs a lane apart: they are the two lines, no guessing needed.
            # This also lets a strip that locked onto the wrong line recover at once
            left, right = columns[0], columns[1]
        else:
            # One blob, or several: each line is the blob nearest to where we expected it
            left = nearest_column(columns, expected_left)
            right = nearest_column(columns, expected_right)

            # The same blob cannot be both lines, and two blobs closer than MIN_LANE_WIDTH
            # are two pieces of one line. Keep whichever is nearer its expected spot
            if left is not None and right is not None and right - left < MIN_LANE_WIDTH:
                if abs(left - expected_left) <= abs(right - expected_right):
                    right = None
                else:
                    left = None

        left_seen[i] = left is not None
        right_seen[i] = right is not None

        # Both lines in view: remember how far apart they are at this distance.
        # One line in view: predict the other one from it, using that remembered width.
        # This is what keeps the car centred when the inside line slides out of the
        # picture in a corner
        if left is not None and right is not None:
            strip_width[i] = right - left
        elif left is not None:
            right = left + strip_width[i]
        elif right is not None:
            left = right - strip_width[i]

        strip_left[i] = left
        strip_right[i] = right
        strip_center[i] = None if left is None else (left + right) // 2

        draw_strip(image, i, top, bottom)

    # The nearest strip that found the lane tells us where the car is; the farthest one
    # tells us where the lane is going
    found = [i for i in range(NUM_STRIPS) if strip_center[i] is not None]
    near_center_col = strip_center[found[0]] if found else None
    far_center_col = strip_center[found[-1]] if found else None

    return image


# [FUNCTION] Draws one strip's result: its top edge, the two line positions (green = seen,
# orange = predicted) and the lane centre (red)
def draw_strip(image, i, top, bottom):
    width = image.shape[1]
    row = (top + bottom) // 2
    cv.line(image, (0, top), (width - 1, top), (0, 215, 255), 1)
    for col, seen in ((strip_left[i], left_seen[i]), (strip_right[i], right_seen[i])):
        if col is None:
            continue
        color = (0, 255, 0) if seen else (0, 140, 255)
        cv.circle(image, (int(rc_utils.clamp(col, 0, width - 1)), row), 6, color, -1)
    if strip_center[i] is not None:
        cv.circle(image, (int(rc_utils.clamp(strip_center[i], 0, width - 1)), row), 5, (0, 0, 255), -1)


# [FUNCTION] Draws the image centre, the lane centre path through the strips and the
# numbers the controller is using
def draw_overlay(image):
    height, width = image.shape[0], image.shape[1]
    band_top = int(height * CROP_TOP_FRAC)
    strip_height = (height - band_top) // NUM_STRIPS

    # The centre of the image (grey). Every offset is measured from here
    cv.line(image, (width // 2, band_top), (width // 2, height), (200, 200, 200), 1)

    # The lane centre path (red), joining the centres found in each strip
    previous = None
    for i in range(NUM_STRIPS):
        if strip_center[i] is None:
            continue
        row = height - i * strip_height - strip_height // 2
        point = (int(rc_utils.clamp(strip_center[i], 0, width - 1)), row)
        if previous is not None:
            cv.line(image, previous, point, (0, 0, 255), 2)
        previous = point

    # The numbers, top left
    text = f"offset {offset:+.2f}   heading {heading:+.2f}   angle {angle:+.2f}   speed {speed:.2f}"
    cv.putText(image, text, (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed, angle, offset, heading, last_offset, integral

    # Initialize variables
    speed = 0.0
    angle = 0.0
    offset = 0.0
    heading = 0.0
    last_offset = 0.0
    integral = 0.0
    for i in range(NUM_STRIPS):
        strip_left[i] = None
        strip_right[i] = None
        strip_center[i] = None
        strip_width[i] = LANE_WIDTH_GUESS

    # Begin at a standstill
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lane Following - Two Blue Lines\n"
        "\n"
        f"Speed is fixed at {SPEED}. Steering = PID on the lane offset in the nearest strip\n"
        f"(KP = {KP}, KI = {KI}, KD = {KD}) + {K_HEADING} x the lane heading from the far strips.\n"
        f"The window shows {NUM_STRIPS} strips: green = line seen, orange = line predicted,\n"
        "red = lane centre. Press Ctrl+C to stop."
    )


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed
def update():
    global speed, angle, offset, heading, last_offset, integral

    # Search for the two lines in every strip of the current colour image
    image = update_lines()

    # If we can see the lane, turn where it is and where it is going into a steering
    # angle. If we cannot, keep the previous angle so a short gap does not straighten
    # the wheels
    if near_center_col is not None and image is not None:
        half_width = image.shape[1] / 2

        # Offset: how far the lane centre is from the image centre in the nearest strip,
        # as a fraction of half the image width. Positive means the lane is to the right
        # of the car, and the car must turn right (positive angle) to get back to it
        offset = (near_center_col - half_width) / half_width

        # Heading: how far the lane centre shifts sideways between the nearest strip and
        # the farthest one. Positive means the lane bends right up ahead. Steering on this
        # starts the turn before the car reaches the bend, instead of waiting for the
        # offset to grow
        heading = (far_center_col - near_center_col) / half_width

        # Integral: the offset added up over time, so a small steady offset eventually
        # adds up to a correction. Clamped so it cannot wind up and saturate the steering
        dt = rc.get_delta_time()
        integral = rc_utils.clamp(integral + offset * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)

        # Derivative: how fast the offset is changing, which resists overshoot
        derivative = (offset - last_offset) / dt if dt > 0 else 0.0
        last_offset = offset

        angle = rc_utils.clamp(
            KP * offset + KI * integral + KD * derivative + K_HEADING * heading, -1.0, 1.0
        )

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
    # Print a line of ascii text for the nearest strip (L, R, lane centre |), which
    # strips saw which lines (near to far), and the numbers the controller is using
    if rc.camera.get_color_image() is None:
        print("X" * 10 + " (No image) " + "X" * 10)
        return

    s = ["-"] * 32
    bucket = max(1, image_width // 32)
    for col, mark in ((strip_left[0], "L"), (strip_right[0], "R"), (strip_center[0], "|")):
        if col is not None:
            s[int(rc_utils.clamp(col // bucket, 0, 31))] = mark

    seen = " ".join(
        ("L" if left_seen[i] else "-") + ("R" if right_seen[i] else "-")
        for i in range(NUM_STRIPS)
    )
    print(
        "".join(s)
        + f" : seen {seen}  offset = {offset:+.2f}  heading = {heading:+.2f}  angle = {angle:+.2f}"
    )


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
