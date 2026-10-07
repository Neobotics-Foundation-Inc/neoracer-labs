"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs

File Name: lane_follow.py

Title: Lane Following - Two Blue Lines

Author: Neobotics

Purpose: Drive the NeoRacer down a lane marked by two lines of blue tape and stay as
close to the middle of the lane as possible, through corners as well as straights.

The bottom half of the camera image is cut into horizontal strips, from the bumper out
to the far end of the view. In every strip the program looks for pieces of TAPE, not
just blue pixels: a piece of tape is a narrow stripe that runs through the strip. The
two stripes the right distance apart are the lane lines. A line is followed from strip
to strip in the direction it leans, so a stripe that does not continue the line below
it is ignored. Where a line is out of view it is predicted: beside the other line at the
lane width that strip measured earlier, or along the direction the line was heading. The
nearest strip says how far off centre the car is. A straight line fitted through the lane
centres of all the strips says where the lane is heading, so the car begins turning
before it reaches a bend. If too few strips see tape, the car holds its last steering
angle instead of chasing noise. Speed is constant for now.

Expected Outcome: When the user runs the script, the RACECAR drives forward at a fixed
speed and steers to keep itself centred between the two blue lines. A window on the
Jetson desktop shows the live camera image with the strips, the blue blobs that were
rejected (grey) and accepted as tape (white), the seen (green) and predicted (orange)
line positions, the lane centre path (red) and the numbers the controller is using.
- Straight lane: drives down the middle
- Corner ahead: the far strips see the lane bend and the car begins to turn early
- One line out of view: its position is predicted, the car stays centred
- Blue clutter that is not tape-shaped or does not continue a line: ignored
- Fewer than MIN_STRIPS_SEEN strips with tape: hold the last steering angle
- Ctrl+C stops the program; the driver zeroes the motor half a second later
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
# The HSV colour range for the blue tape, as (hsv_min, hsv_max), tuned on the car with
# the HSV tuner on 2026-10-07. OpenCV hue runs 0..179. The range is wide in hue and low
# in saturation because that is how the tape reads under the lab lights; a different
# floor or lighting may need a retune. If the mask misses the tape, widen the range; if
# it grabs the floor, tighten it. utility/hsv-p_tuner.py or the linefollow dashboard
# (port 8086) show the mask live while you drag sliders.
BLUE = ((31, 40, 125), (117, 128, 255))

# The smallest blob (in pixels, inside one strip) we even look at
MIN_CONTOUR_AREA = 60

# What a piece of tape looks like inside one strip: a stripe between these two widths
# (pixels, measured across the stripe), at least MIN_ASPECT times longer than it is wide,
# that runs through at least MIN_LENGTH_FRAC of the strip's height. A stripe may be
# shorter if it touches the left or right edge of the image, which is where a line
# leaves the view in a corner. Blobs that fail this are blue, but they are not tape.
# Measure your tape in the window if these need changing
TAPE_WIDTH_MIN = 6
TAPE_WIDTH_MAX = 35
MIN_ASPECT = 2.0
MIN_LENGTH_FRAC = 0.6

# How much of the image we search, as a fraction of the height measured from the top.
# 0.5 means "the bottom half of the image": the top half is wall and horizon, not floor.
CROP_TOP_FRAC = 0.5

# The searched area is cut into this many horizontal strips. Strip 0 is nearest the car
# and gives the lateral position; the whole set gives the heading
NUM_STRIPS = 4

# A line may not be found more than this many pixels from where it is expected: where
# the stripe in the strip below points to, or where it was last frame. A stripe farther
# away than this does not continue the line, so it is not the line
MAX_JUMP = 100

# Two stripes are the two lane lines only if their distance apart is right. Before a
# strip has measured its lane width, anything between these two values is plausible
MIN_LANE_WIDTH = 100
MAX_LANE_WIDTH = 600
# After the strip has measured the width, a pair must match it this closely (fraction)
WIDTH_TOLERANCE = 0.25

# Lane width in pixels before the car has measured it. Each strip replaces this with its
# own measurement the first time it sees both lines (far strips measure smaller widths)
LANE_WIDTH_GUESS = 400

# A stripe leaning more than this many columns per row is nearly horizontal; its lean is
# clamped so one odd blob cannot throw the expectation across the whole image
MAX_TILT = 3.0

# We trust the lane only when at least this many strips actually see tape. Fewer, and the
# car holds its last steering angle rather than steering on a guess
MIN_STRIPS_SEEN = 2

# After this many frames without a trusted lane, forget where the lines were and start
# the search again from the image centre
LOST_FRAMES = 30

# Smoothing of the heading estimate, 0..1. Smaller = smoother but slower to react
HEADING_SMOOTH = 0.3

# How fast to drive. Constant for now. The number is a fraction of full scale, and full
# scale is the library's max speed (1.0) x the driver's throttle cap (max_speed_forward
# in throttle.yaml) x the firmware's 6 m/s ceiling, so with the shipped settings 0.1 is
# roughly 0.6 m/s. Start low and raise it once the steering is tuned.
SPEED = 0.1

# Steering gains. Both inputs are normalised to -1..1 across half the image width.
#   offset:  where the lane centre is in the nearest strip, relative to the image centre.
#            Positive = lane is to the right of the car.
#   heading: how far the lane centre moves sideways from the nearest strip to the
#            farthest one. Positive = the lane bends to the right up ahead.
# The PID on offset keeps the car centred. K_HEADING adds steering for the bend that is
# coming, before the offset has had a chance to grow: that is the prediction.
# KP, KI and KD were tuned on the car on 2026-10-07.
KP = 2.0           # proportional: steer in proportion to how far off centre the car is
KI = 0.0           # integral: slowly removes a steady offset. Leave at 0 unless the car sits off centre
KD = 0.2           # derivative: damps the swing when the offset is changing quickly
K_HEADING = 0.6    # feed-forward: how hard to pre-steer for the bend the strips see
INTEGRAL_LIMIT = 0.5  # the integral term can never ask for more than this much steering

# >> Variables
speed = 0.0        # The current speed of the car
angle = 0.0        # The current angle of the car's wheels
offset = 0.0       # Lane centre offset in the nearest strip, -1..1
heading = 0.0      # Sideways drift of the lane centre from near strip to far strip, -1..1
last_offset = 0.0  # The offset from the previous frame, for the derivative term
integral = 0.0     # The running sum of offset over time, for the integral term
lane_found = False # True when enough strips see tape to trust the lane this frame
lost_count = 0     # Frames in a row without a trusted lane
image_width = 640  # The width of the last camera frame, for the update_slow print-out

# Per-strip results, index 0 = nearest the car. Columns are pixel columns in the image;
# tilts are how many columns a line shifts per row going up the image (negative = leans
# left). Memory holds last frame's columns and is where the search starts next frame
strip_left = [None] * NUM_STRIPS     # Column of the left line (seen or predicted)
strip_right = [None] * NUM_STRIPS    # Column of the right line (seen or predicted)
left_tilt = [0.0] * NUM_STRIPS       # Lean of the left line in each strip
right_tilt = [0.0] * NUM_STRIPS      # Lean of the right line in each strip
left_seen = [False] * NUM_STRIPS     # True where the left line was actually in view
right_seen = [False] * NUM_STRIPS    # True where the right line was actually in view
strip_center = [None] * NUM_STRIPS   # Column of the lane centre in each strip
strip_width = [LANE_WIDTH_GUESS] * NUM_STRIPS  # Lane width each strip last measured
width_known = [False] * NUM_STRIPS   # True once the strip has measured its lane width
memory_left = [None] * NUM_STRIPS    # Where the left line was last frame
memory_right = [None] * NUM_STRIPS   # Where the right line was last frame


########################################################################################
# Functions
########################################################################################

# [FUNCTION] How many columns a tape stripe shifts per row going UP the image (toward the
# far end), from a straight line fitted through the blob. Negative = it leans left
def segment_tilt(contour):
    vx, vy, _, _ = cv.fitLine(contour, cv.DIST_L2, 0, 0.01, 0.01).flatten()
    if abs(vy) < 0.05:
        return 0.0  # lying almost flat: its lean says nothing useful about the next strip
    # Along the fitted direction, one row up (dy = -1) moves the column by -vx / vy
    return float(rc_utils.clamp(-vx / vy, -MAX_TILT, MAX_TILT))


# [FUNCTION] Finds the blue blobs in one strip that are shaped like tape. Returns a list
# of (column, tilt), one per accepted stripe. Every blob is outlined grey, every accepted
# stripe white, so the window shows what was rejected and why the controller is confident
def tape_segments(strip, strip_height, width):
    segments = []
    for contour in rc_utils.find_contours(strip, BLUE[0], BLUE[1]):
        if rc_utils.get_contour_area(contour) < MIN_CONTOUR_AREA:
            continue
        rc_utils.draw_contour(strip, contour, rc_utils.ColorBGR.dark_gray.value)

        # The tightest rotated box around the blob: its short side is the width of the
        # tape, its long side is how much tape is inside this strip
        _, (side_a, side_b), _ = cv.minAreaRect(contour)
        tape_width = min(side_a, side_b)
        tape_length = max(side_a, side_b)
        x, _, w, _ = cv.boundingRect(contour)
        touches_edge = x <= 0 or x + w >= width

        if not TAPE_WIDTH_MIN <= tape_width <= TAPE_WIDTH_MAX:
            continue
        if tape_length < MIN_ASPECT * tape_width:
            continue
        if tape_length < MIN_LENGTH_FRAC * strip_height and not touches_edge:
            continue
        center = rc_utils.get_contour_center(contour)
        if center is None:
            continue

        rc_utils.draw_contour(strip, contour, rc_utils.ColorBGR.white.value)
        segments.append((center[1], segment_tilt(contour)))
    return segments


# [FUNCTION] The segment nearest to an expected column, or None if the nearest one is
# more than MAX_JUMP away (then it does not continue this line)
def nearest_segment(segments, expected):
    best = None
    for segment in segments:
        if best is None or abs(segment[0] - expected) < abs(best[0] - expected):
            best = segment
    if best is None or abs(best[0] - expected) > MAX_JUMP:
        return None
    return best


# [FUNCTION] Two segments the right distance apart to be the two lane lines, as
# (left, right), or None. Once the strip knows its lane width a pair must match it within
# WIDTH_TOLERANCE; before that any plausible lane width will do. If several pairs fit,
# the one whose middle is nearest the expected lane centre wins
def lane_pair(segments, i, expected_center):
    pairs = []
    for a in segments:
        for b in segments:
            gap = b[0] - a[0]
            if width_known[i]:
                fits = abs(gap - strip_width[i]) <= WIDTH_TOLERANCE * strip_width[i]
            else:
                fits = MIN_LANE_WIDTH <= gap <= MAX_LANE_WIDTH
            if fits:
                pairs.append((a, b))
    if not pairs:
        return None
    return min(pairs, key=lambda p: abs((p[0][0] + p[1][0]) / 2 - expected_center))


# [FUNCTION] Where to expect a line in strip i. First choice: where the stripe found in
# the strip below points to (continuity: the line continues in the direction it leans).
# Second: where the line was last frame. Third: the fallback
def expected_column(cols, tilts, memory, i, strip_height, fallback):
    if i > 0 and cols[i - 1] is not None:
        return cols[i - 1] + tilts[i - 1] * strip_height
    if memory[i] is not None:
        return memory[i]
    return fallback


# [FUNCTION] Fills in the strips where a line was not seen. First choice: beside the
# other line, a lane width away, when that other line was actually seen there (the two
# lines are parallel and the width is measured). Second: carry on along the direction the
# line was heading from the nearest strip that did see it. Otherwise it stays unknown
def predict_missing(cols, tilts, seen, other_cols, other_seen, sign, strip_height):
    seen_at = [i for i in range(NUM_STRIPS) if seen[i]]
    for i in range(NUM_STRIPS):
        if seen[i]:
            continue
        if other_seen[i]:
            cols[i] = other_cols[i] + sign * strip_width[i]
            tilts[i] = 0.0
        elif seen_at:
            j = min(seen_at, key=lambda k: abs(k - i))
            cols[i] = cols[j] + tilts[j] * (i - j) * strip_height
            tilts[i] = tilts[j]
        else:
            cols[i] = None
            tilts[i] = 0.0


# [FUNCTION] Finds the two tape lines in every strip of the current colour image, fills
# in the per-strip results and decides whether the lane can be trusted. Returns the image
# with the detections drawn on it, or None if the camera has not delivered a frame yet
def update_lines():
    global image_width, lane_found, lost_count, heading

    image = rc.camera.get_color_image()

    # The camera node may not have published yet on the first few frames after start()
    if image is None:
        for i in range(NUM_STRIPS):
            strip_center[i] = None
        lane_found = False
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

        # Where each line should be, from the strip below, else last frame, else a lane
        # width either side of the image centre
        expected_left = expected_column(
            strip_left, left_tilt, memory_left, i, strip_height, width // 2 - strip_width[i] // 2)
        expected_right = expected_column(
            strip_right, right_tilt, memory_right, i, strip_height, width // 2 + strip_width[i] // 2)

        # Everything in this strip that is shaped like tape
        segments = tape_segments(strip, strip_height, width)

        # Best case: two stripes the right distance apart. Otherwise each line is the
        # stripe that continues it (nearest to where it is expected, within MAX_JUMP)
        pair = lane_pair(segments, i, (expected_left + expected_right) / 2)
        if pair is not None:
            left, right = pair
        else:
            left = nearest_segment(segments, expected_left)
            right = nearest_segment(segments, expected_right)
            # The same stripe cannot be both lines, and two stripes closer together than
            # a lane cannot be the two lines. Keep whichever is nearer its expected spot
            if left is not None and right is not None and right[0] - left[0] < MIN_LANE_WIDTH:
                if abs(left[0] - expected_left) <= abs(right[0] - expected_right):
                    right = None
                else:
                    left = None

        left_seen[i] = left is not None
        right_seen[i] = right is not None
        strip_left[i], left_tilt[i] = (left if left is not None else (None, 0.0))
        strip_right[i], right_tilt[i] = (right if right is not None else (None, 0.0))

        # Both lines in view: remember how far apart they are at this distance
        if left is not None and right is not None:
            strip_width[i] = right[0] - left[0]
            width_known[i] = True

    # Predict the lines where they were out of view, then mark the lane centre per strip
    predict_missing(strip_left, left_tilt, left_seen, strip_right, right_seen, -1, strip_height)
    predict_missing(strip_right, right_tilt, right_seen, strip_left, left_seen, +1, strip_height)
    for i in range(NUM_STRIPS):
        if strip_left[i] is None or strip_right[i] is None:
            strip_center[i] = None
        else:
            strip_center[i] = (strip_left[i] + strip_right[i]) / 2

    # The lane is trusted only when enough strips actually saw tape
    seen_strips = [i for i in range(NUM_STRIPS) if left_seen[i] or right_seen[i]]
    lane_found = len(seen_strips) >= MIN_STRIPS_SEEN

    # Heading: a straight line fitted through the lane centres of the strips that saw
    # tape says how far the centre shifts from the near strip to the far strip. With one
    # strip, the lean of the tape in it says the same thing. Smoothed over frames
    half_width = width / 2
    new_heading = None
    if len(seen_strips) >= 2:
        slope = np.polyfit(seen_strips, [strip_center[i] for i in seen_strips], 1)[0]
        new_heading = slope * (NUM_STRIPS - 1) / half_width
    elif len(seen_strips) == 1:
        i = seen_strips[0]
        tilts = [t for t, s in ((left_tilt[i], left_seen[i]), (right_tilt[i], right_seen[i])) if s]
        new_heading = (sum(tilts) / len(tilts)) * (NUM_STRIPS - 1) * strip_height / half_width
    if new_heading is not None:
        heading += HEADING_SMOOTH * (rc_utils.clamp(new_heading, -1.0, 1.0) - heading)

    # Memory for next frame: where the lines are now. After LOST_FRAMES without a trusted
    # lane, forget them so the search restarts from the image centre
    if lane_found:
        lost_count = 0
        for i in range(NUM_STRIPS):
            memory_left[i] = strip_left[i]
            memory_right[i] = strip_right[i]
    else:
        lost_count += 1
        if lost_count >= LOST_FRAMES:
            for i in range(NUM_STRIPS):
                memory_left[i] = None
                memory_right[i] = None

    for i in range(NUM_STRIPS):
        draw_strip(image, i, height - (i + 1) * strip_height, height - i * strip_height)
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
    status = "LANE" if lane_found else "NO LANE"
    text = f"{status}   offset {offset:+.2f}   heading {heading:+.2f}   angle {angle:+.2f}   speed {speed:.2f}"
    cv.putText(image, text, (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed, angle, offset, heading, last_offset, integral, lane_found, lost_count

    # Initialize variables
    speed = 0.0
    angle = 0.0
    offset = 0.0
    heading = 0.0
    last_offset = 0.0
    integral = 0.0
    lane_found = False
    lost_count = 0
    for i in range(NUM_STRIPS):
        strip_left[i] = None
        strip_right[i] = None
        strip_center[i] = None
        left_tilt[i] = 0.0
        right_tilt[i] = 0.0
        left_seen[i] = False
        right_seen[i] = False
        strip_width[i] = LANE_WIDTH_GUESS
        width_known[i] = False
        memory_left[i] = None
        memory_right[i] = None

    # Begin at a standstill
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(
        ">> Lane Following - Two Blue Lines\n"
        "\n"
        f"Speed is fixed at {SPEED}. Steering = PID on the lane offset in the nearest strip\n"
        f"(KP = {KP}, KI = {KI}, KD = {KD}) + {K_HEADING} x the lane heading fitted through all strips.\n"
        f"The window shows {NUM_STRIPS} strips: grey = blue blob rejected, white = tape stripe,\n"
        "green = line seen, orange = line predicted, red = lane centre. Press Ctrl+C to stop."
    )


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed
def update():
    global speed, angle, offset, last_offset, integral

    # Search for the two lines in every strip of the current colour image
    image = update_lines()

    # If the lane is trusted, turn where it is and where it is going into a steering
    # angle. If it is not, keep the previous angle rather than steer on a guess
    if lane_found and strip_center[0] is not None:
        half_width = image.shape[1] / 2

        # Offset: how far the lane centre is from the image centre in the nearest strip,
        # as a fraction of half the image width. Positive means the lane is to the right
        # of the car, and the car must turn right (positive angle) to get back to it
        offset = (strip_center[0] - half_width) / half_width

        # Integral: the offset added up over time, so a small steady offset eventually
        # adds up to a correction. Clamped so it cannot wind up and saturate the steering
        dt = rc.get_delta_time()
        integral = rc_utils.clamp(integral + offset * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)

        # Derivative: how fast the offset is changing, which resists overshoot
        derivative = (offset - last_offset) / dt if dt > 0 else 0.0
        last_offset = offset

        # heading was computed in update_lines() from where the lane is going
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
    # No frame has arrived on /camera/color yet. That is the camera node, not this
    # script: check `racecar service status` and `ros2 topic hz /camera/color`
    if rc.camera.get_color_image() is None:
        print("X" * 10 + " (No image from /camera/color: is the camera node running?) " + "X" * 10)
        return

    # One line of ascii text for the nearest strip (L, R, lane centre |), which strips saw
    # which lines (near to far), and the numbers the controller is using
    s = ["-"] * 32
    bucket = max(1, image_width // 32)
    for col, mark in ((strip_left[0], "L"), (strip_right[0], "R"), (strip_center[0], "|")):
        if col is not None:
            s[int(rc_utils.clamp(col // bucket, 0, 31))] = mark

    seen = " ".join(
        ("L" if left_seen[i] else "-") + ("R" if right_seen[i] else "-")
        for i in range(NUM_STRIPS)
    )
    status = "LANE   " if lane_found else "NO LANE"
    print(
        "".join(s)
        + f" : {status} seen {seen}  offset = {offset:+.2f}  heading = {heading:+.2f}  angle = {angle:+.2f}"
    )


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
