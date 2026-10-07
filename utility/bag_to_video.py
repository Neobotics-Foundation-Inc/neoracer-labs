#!/usr/bin/env python3
"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-outreach-labs

File Name: bag_to_video.py

Title: Rosbag to Video

Purpose: Turn the camera frames recorded in a rosbag into an ordinary mp4 file that plays
in QuickTime or VLC. The NeoRacer camera node publishes each frame as the raw JPEG bytes
inside a sensor_msgs/Image with encoding "jpeg", so a bag of /camera/color is a list of
JPEGs with timestamps. This reads them straight out of the bag (no playback needed),
decodes each one and writes the video at the frame rate they were really recorded at.

Usage, on the car:
    ros2 bag record -o lane_run_1 /camera/color /drive /odom     # while driving
    python3 bag_to_video.py lane_run_1                            # -> lane_run_1.mp4
    python3 bag_to_video.py lane_run_1 my_name.mp4                # choose the file name

Then copy the mp4 to your laptop:
    scp racecar@neoracer:~/lane_run_1.mp4 ~/Downloads/
"""

########################################################################################
# Imports
########################################################################################

import sys

import cv2 as cv
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

########################################################################################
# Constants
########################################################################################

CAMERA_TOPIC = "/camera/color"

########################################################################################
# Main
########################################################################################

if len(sys.argv) < 2:
    print("Usage: python3 bag_to_video.py <bag folder> [output.mp4]")
    sys.exit(1)

bag_folder = sys.argv[1].rstrip("/")
output = sys.argv[2] if len(sys.argv) > 2 else bag_folder + ".mp4"

# Open the bag for reading, oldest message first
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=bag_folder, storage_id="sqlite3"),
    rosbag2_py.ConverterOptions("", ""),
)

# Pull out every camera frame and the time it was recorded
frames = []
stamps = []
while reader.has_next():
    topic, data, stamp_ns = reader.read_next()
    if topic != CAMERA_TOPIC:
        continue
    message = deserialize_message(data, Image)
    frame = cv.imdecode(np.frombuffer(bytes(message.data), np.uint8), cv.IMREAD_COLOR)
    if frame is not None:
        frames.append(frame)
        stamps.append(stamp_ns / 1e9)

if len(frames) < 2:
    print(f"No camera frames on {CAMERA_TOPIC} in {bag_folder}")
    sys.exit(1)

# The real recording rate, so the video plays back at the speed the car drove
fps = (len(frames) - 1) / (stamps[-1] - stamps[0])
height, width = frames[0].shape[0], frames[0].shape[1]

writer = cv.VideoWriter(output, cv.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
for frame in frames:
    writer.write(frame)
writer.release()

print(f"Wrote {output}: {len(frames)} frames, {width}x{height}, {fps:.1f} fps, "
      f"{stamps[-1] - stamps[0]:.1f} seconds")
