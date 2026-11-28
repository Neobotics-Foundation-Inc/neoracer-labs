"""
Neobotics Foundation Inc.
GPLv3 License
neoracer-labs/lab_d/

File Name: lab_d.py

Title: Lab D - Driving in Shapes

Author: [PLACEHOLDER] << [Write your name or team name here]

Version: 0.9.0

Purpose: Create a script to enable semi-autonomous driving for the NeoRacer. Flips of the
handset's mode switch start a series of instructions sent to the NeoRacer, which make it
drive in various shapes. Complete the lines of code under the #TODO indicators to complete
the lab.

Expected Outcome: When the user runs the script, they are able to control the NeoRacer
with the handset's mode switch:
- The first time the mode switch is flipped down, drive through the obstacle "Zigzag"
- The second time, drive through the obstacle "Spiral"
- The third time, drive through the obstacle "Hallway"
- The fourth time, drive through the obstacle "Maze", then start over with the "Zigzag"
- When the mode switch is flipped up, the handset drives the car and the queue is cleared
"""

########################################################################################
# Imports
########################################################################################

import racecar_core

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# A queue of driving steps to execute
# Each entry is a list containing (time remaining, speed, angle)
queue = []

# Which shape the next flip of the mode switch drives: 0 Zigzag, 1 Spiral, 2 Hallway, 3 Maze
shape_index = 0


########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once when the program starts
def start():
    # Begin at a full stop
    rc.drive.stop()

    # Begin with an empty queue
    queue.clear()

    # The NeoRacer's full speed (1.0) is about 6 m/s. Uncomment the next line and pick a
    # lower limit for your room. Every speed the program sets is then scaled by it
    # rc.drive.set_max_speed(0.25)

    # Print start message
    print(
        ">> Lab D - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "   Mode switch down = drive the next shape: \"Zigzag\", \"Spiral\", \"Hallway\", \"Maze\"\n"
        "   Mode switch up = stop and clear the queue\n"
    )


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until Ctrl+C is pressed
# in the terminal
def update():
    global queue
    global shape_index

    # Each time the mode switch is flipped down, add the instructions for the next shape.
    # On the NeoRacer, Button.RB reads as down while the switch is down, and the car follows
    # the program only in that position
    if rc.controller.was_pressed(rc.controller.Button.RB):
        if shape_index == 0:
            drive_zigzag()
        elif shape_index == 1:
            drive_spiral()
        elif shape_index == 2:
            drive_hallway()
        else:
            drive_maze()
        shape_index = (shape_index + 1) % 4

    # When the mode switch is flipped up, the handset drives the car again. Clear the queue
    # so the shape does not continue when the switch comes back down
    if rc.controller.was_released(rc.controller.Button.RB):
        queue.clear()

    # TODO Part 1: Analyze the following code segment that executes instructions from the queue.
    # Fill in the blanks with the missing variable assignments and indices according to the
    # behavior described by the comment below.

    # With an empty queue, the car stands still
    speed = 0
    angle = 0

    # If the queue is not empty, follow the current drive instruction
    if len(queue) > 0:
        speed = _____
        angle = _____
        queue[_][_] -= rc.get_delta_time()
        if queue[_][_] <= 0:
            queue.pop(0)

    # Send speed and angle commands to the NeoRacer
    rc.drive.set_speed_angle(speed, angle)


# [FUNCTION] When the function is called, clear the queue, then place instructions
# inside of the queue that cause the NeoRacer to drive in the zigzag
def drive_zigzag():
    global queue

    # Use this section to define and tune static variables

    queue.clear()

    # TODO Part 2: Append the correct variables in the correct order in order
    # for the NeoRacer to drive in the "Zigzag" obstacle course
    # [Hint] queue.append([time, speed, angle])


# [FUNCTION] When the function is called, clear the queue, then place instructions
# inside of the queue that cause the NeoRacer to drive in the spiral
def drive_spiral():
    global queue

    # Use this section to define and tune static variables

    queue.clear()

    # TODO Part 3: Append the instructions into the queue that represent the NeoRacer
    # driving in the "Spiral" obstacle course


# [FUNCTION] When the function is called, clear the queue, then place instructions 
# inside of the queue that cause the NeoRacer to drive through the hallway
def drive_hallway():
    global queue

    # TODO Part 4: Create constants that represent the NeoRacer driving through
    # the "Hallway" obstacle course, and then append the instructions in the
    # correct order into the queue for execution

    queue.clear()


# [FUNCTION] When the function is called, clear the queue, then place instructions 
# inside of the queue that cause the NeoRacer to drive in the maze
def drive_maze():
    global queue

    # TODO Part 5: Create constants that represent the NeoRacer driving through
    # different parts of the maze, and then append the instructions in the
    # correct order into the queue for execution

    queue.clear()


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
