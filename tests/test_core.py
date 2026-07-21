"""
MIT BWSI Autonomous RACECAR Course
GNU General Public License v3.0
racecar-neo-v2 / labs / tests

File Name: test_core.py

Title: Test Core

Purpose: A simple program which can be used to manually test racecar_core functionality.
"""

########################################################################################
# Imports
########################################################################################

import math

import cv2 as cv
import numpy as np

import racecar_core

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Single fullscreen OpenCV window shared by all three display modes so we
# don't leak orphan windows when the active mode changes.
WINDOW_NAME = "test_core"

# Drive tuning.
MAX_SPEED_STEP = 0.05
MAX_SPEED_MIN = 0.0
MAX_SPEED_MAX = 1.0

# LIDAR top-down render config.
LIDAR_RADIUS_PX = 400
LIDAR_MAX_RANGE_CM = 1000

max_speed = 0.0

# Active display mode: None (window closed) or one of "camera", "edgetpu", "lidar".
# Buttons toggle their own mode on was_pressed; pressing a different mode's
# button while one is active switches to that mode (single window, single mode).
display_mode = None

########################################################################################
# Functions
########################################################################################


def _open_window():
    """Create the shared fullscreen OpenCV window. Idempotent."""
    cv.namedWindow(WINDOW_NAME, cv.WINDOW_NORMAL)
    cv.setWindowProperty(WINDOW_NAME, cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)


def _close_window():
    """Tear down the shared window if it exists. Safe to call when nothing is open."""
    cv.destroyWindow(WINDOW_NAME)
    # destroyWindow only queues the teardown — pump the event loop so the
    # window actually disappears before update() returns.
    cv.waitKey(1)


def _show(image):
    """Push a frame to the shared fullscreen window."""
    cv.imshow(WINDOW_NAME, image)
    cv.waitKey(1)


def _toggle_mode(button_mode):
    """Toggle off if button_mode is already active, else switch to it."""
    global display_mode
    if display_mode == button_mode:
        display_mode = None
        _close_window()
    else:
        was_closed = display_mode is None
        display_mode = button_mode
        if was_closed:
            _open_window()


def _annotate_detections(image, detections):
    """Return a copy of image with EdgeTPU bounding boxes / labels drawn on."""
    annotated = image.copy()
    for det in detections:
        cx, cy, w, h = det.bbox
        x1, y1 = int(cx - w / 2), int(cy - h / 2)
        x2, y2 = int(cx + w / 2), int(cy + h / 2)
        cv.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv.putText(
            annotated,
            f"{det.class_id} {det.score:.0%}",
            (x1, max(y1 - 8, 0)),
            cv.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
    return annotated


def _render_lidar(samples):
    """Render a top-down LIDAR scan to a BGR image (mirrors display.show_lidar)."""
    image = np.zeros((2 * LIDAR_RADIUS_PX, 2 * LIDAR_RADIUS_PX, 3), np.uint8)
    n = len(samples)
    for i in range(n):
        d = samples[i]
        if 0 < d < LIDAR_MAX_RANGE_CM:
            angle = 2 * math.pi * i / n
            length = LIDAR_RADIUS_PX * d / LIDAR_MAX_RANGE_CM
            r = int(LIDAR_RADIUS_PX - length * math.cos(angle))
            c = int(LIDAR_RADIUS_PX + length * math.sin(angle))
            image[r, c, 2] = 255
    cv.circle(image, (LIDAR_RADIUS_PX, LIDAR_RADIUS_PX), 3, (0, 255, 0), -1)
    return image


def start():
    """
    This function is run once every time the start button is pressed.
    """
    global max_speed
    global display_mode

    max_speed = 0.25
    rc.drive.set_max_speed(max_speed)
    rc.drive.stop()

    # Make sure no window is left over from a previous run.
    if display_mode is not None:
        _close_window()
        display_mode = None

    print(
        ">> Test Core: A testing program for the racecar_core library.\n"
        "\n"
        "Controls:\n"
        "    Left joystick Y = drive forward / backward\n"
        "    Right joystick X = steer left / right\n"
        "    Left joystick click = decrease max speed by 0.05\n"
        "    Right joystick click = increase max speed by 0.05\n"
        "    A button = toggle forward camera (fullscreen)\n"
        "    B button = toggle forward camera + EdgeTPU overlay (fullscreen)\n"
        "    X button = toggle LIDAR scan (fullscreen)\n"
        "    Y button = hold to print IMU data (closes any open window)\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed.
    """
    global max_speed
    global display_mode

    # Drive: left stick Y for throttle, right stick X for steering.
    # Left/right bumpers are intentionally unbound — the driver mux owns them.
    _, left_y = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    right_x, _ = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)
    rc.drive.set_speed_angle(left_y, right_x)

    # Adjust max speed in 0.05 steps on joystick click. Print once per change.
    if rc.controller.was_pressed(rc.controller.Button.RJOY):
        new_speed = min(MAX_SPEED_MAX, round(max_speed + MAX_SPEED_STEP, 2))
        if new_speed != max_speed:
            max_speed = new_speed
            rc.drive.set_max_speed(max_speed)
            print(f"max_speed increased to [{max_speed:.2f}]")
    if rc.controller.was_pressed(rc.controller.Button.LJOY):
        new_speed = max(MAX_SPEED_MIN, round(max_speed - MAX_SPEED_STEP, 2))
        if new_speed != max_speed:
            max_speed = new_speed
            rc.drive.set_max_speed(max_speed)
            print(f"max_speed decreased to [{max_speed:.2f}]")

    # Display-mode toggles. Each button toggles its own view on was_pressed;
    # pressing a different display button while one is active switches modes.
    # Y is hold-to-print instead of a toggle (handled below) since the IMU
    # stream is one line per frame and would flood the terminal otherwise.
    if rc.controller.was_pressed(rc.controller.Button.A):
        _toggle_mode("camera")
    if rc.controller.was_pressed(rc.controller.Button.B):
        _toggle_mode("edgetpu")
    if rc.controller.was_pressed(rc.controller.Button.X):
        _toggle_mode("lidar")

    # Y: hold to stream IMU data. On the initial press, close any open
    # display window so the terminal output is the only feedback surface.
    if rc.controller.was_pressed(rc.controller.Button.Y) and display_mode is not None:
        _close_window()
        display_mode = None
    if rc.controller.is_down(rc.controller.Button.Y):
        a = rc.physics.get_linear_acceleration()
        w = rc.physics.get_angular_velocity()
        print(
            f"Linear acceleration: ({a[0]:5.2f},{a[1]:5.2f},{a[2]:5.2f}); "
            f"Angular velocity: ({w[0]:5.2f},{w[1]:5.2f},{w[2]:5.2f})"
        )

    # Render whichever display mode is currently active.
    if display_mode == "camera":
        image = rc.camera.get_color_image()
        if image is not None:
            _show(image)
    elif display_mode == "edgetpu":
        image = rc.camera.get_color_image()
        if image is not None:
            _show(_annotate_detections(image, rc.vision.get_detections()))
    elif display_mode == "lidar":
        lidar = rc.lidar.get_samples()
        if lidar is not None and len(lidar) > 0:
            _show(_render_lidar(lidar))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
