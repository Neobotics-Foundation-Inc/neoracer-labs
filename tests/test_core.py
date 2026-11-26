"""
Neobotics Foundation Inc.
GPLv3 License
neoracer-labs/tests/

File Name: test_core.py

Title: Test Core

Author: Neobotics

Version: 0.9.0

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

# Drive tuning. 0.25 of full scale is about 1.5 m/s on the NeoRacer.
MAX_SPEED = 0.25

# LIDAR top-down render config.
LIDAR_RADIUS_PX = 400
LIDAR_MAX_RANGE_CM = 1000

# Active display mode: None (window closed) or one of "camera", "detections", "lidar",
# "imu". Each flip of the handset's mode switch to down advances to the next mode
# (single window, single mode). "imu" closes the window and prints once per second.
MODES = [None, "camera", "detections", "lidar", "imu"]
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
    # destroyWindow only queues the teardown. Pump the event loop so the
    # window actually disappears before update() returns.
    cv.waitKey(1)


def _show(image):
    """Push a frame to the shared fullscreen window."""
    cv.imshow(WINDOW_NAME, image)
    cv.waitKey(1)


def _next_mode():
    """Advance to the next display mode, opening or closing the window as needed."""
    global display_mode
    previous = display_mode
    display_mode = MODES[(MODES.index(previous) + 1) % len(MODES)]
    needs_window = display_mode in ("camera", "detections", "lidar")
    had_window = previous in ("camera", "detections", "lidar")
    if needs_window and not had_window:
        _open_window()
    elif had_window and not needs_window:
        _close_window()


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
    This function is run once when the program starts.
    """
    global display_mode

    rc.drive.set_max_speed(MAX_SPEED)
    rc.drive.stop()

    # Make sure no window is left over from a previous run.
    if display_mode is not None:
        _close_window()
        display_mode = None

    print(
        ">> Test Core: A testing program for the racecar_core library.\n"
        "\n"
        "Controls:\n"
        "    Mode switch down = the program drives from the sticks\n"
        "    Throttle stick (left stick Y) = drive forward / backward\n"
        "    Steering stick (right stick X) = steer left / right\n"
        "    Each flip of the mode switch down = next view: forward camera (fullscreen),\n"
        "        camera + detections overlay, lidar scan, IMU data once per second, off\n"
        f"    Max speed is fixed at {MAX_SPEED}, about {MAX_SPEED * 6:.1f} m/s\n"
    )


def update():
    """
    After start() is run, this function is run every frame until Ctrl+C is pressed
    in the terminal.
    """
    global display_mode

    # Drive: throttle stick (left stick Y) for speed, steering stick (right stick X) for
    # the angle. On the NeoRacer, LB and RB are the handset's mode switch: RB reads as down
    # while the switch is down and the car follows the program; LB reads as down while the
    # switch is up or in the middle and the handset drives the car directly.
    _, left_y = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    right_x, _ = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)
    rc.drive.set_speed_angle(left_y, right_x)

    # Each flip of the mode switch to down advances the view. The sticks keep driving.
    if rc.controller.was_pressed(rc.controller.Button.RB):
        _next_mode()
        print(f"view: {display_mode or 'off'}")

    # Render whichever display mode is currently active.
    if display_mode == "camera":
        image = rc.camera.get_color_image()
        if image is not None:
            _show(image)
    elif display_mode == "detections":
        image = rc.camera.get_color_image()
        if image is not None:
            _show(_annotate_detections(image, rc.vision.get_detections()))
    elif display_mode == "lidar":
        lidar = rc.lidar.get_samples()
        if lidar is not None and len(lidar) > 0:
            _show(_render_lidar(lidar))


def update_slow():
    """
    Runs once per second. Prints the IMU data while the "imu" view is active, so the
    terminal is not flooded with one line per frame.
    """
    if display_mode == "imu":
        a = rc.physics.get_linear_acceleration()
        w = rc.physics.get_angular_velocity()
        # REP-103 body axes; at rest, acceleration z reads +9.81
        print(
            f"Linear acceleration (x fwd, y left, z up): ({a[0]:5.2f},{a[1]:5.2f},{a[2]:5.2f}); "
            f"Angular velocity (roll, pitch, yaw): ({w[0]:5.2f},{w[1]:5.2f},{w[2]:5.2f})"
        )


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
