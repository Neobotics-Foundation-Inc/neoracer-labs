# NeoRacer Labs

Version 0.9.0

Repository: https://github.com/Neobotics-Foundation-Inc/neoracer-labs

These are the student labs for the NeoRacer. They run with the NeoRacer driver 0.9.0 and the student library racecar-neo-library at its tag v0.9.0-neobotics. On a car they live in `~/jupyter_ws/neoracer-os/labs`.

## Files

| Path | Description |
| --- | --- |
| lab_a/lab_a.py | Lab A. Print statements and learn start(), update() and update_slow(). A template with blanks to fill in |
| lab_b/lab_b.py | Lab B. Print statements from the handset's inputs. Template |
| lab_c/lab_c.py | Lab C. Drive the car from the handset's sticks. Template |
| lab_d/lab_d.py | Lab D. Drive in shapes from a queue of instructions. Template |
| lab_e/lab_e.py | Lab E. React to a stoplight with the camera. Template |
| lab_f/lab_f.py | Lab F. Follow a colored line with the camera. Template |
| lab_g/lab_g.py | Lab G. Find an orange cone and park 30 cm from it. Template |
| lab_i/lab_i.py | Lab I. Follow a wall with the lidar. A skeleton to fill in |
| grand_prix/grand_prix.py | Grand Prix. A skeleton for the final course |
| demo.py | A short program that shows start(), update() and update_slow() |
| template.py | The file to copy when you start a new program |
| tests/test_core.py | Drives the car from the sticks and shows the camera, the detections, the lidar and the IMU, one view per flip of the mode switch |
| tests/test_slam.py | Checks rc.slam against the car's autonomy service: status, a drive, a map save and a lift |
| tests/test_async_core_real.ipynb | A notebook that checks every sensor and control, one section at a time, and prints a summary |
| utility/hsv-p_tuner.py | A window for tuning a color range and a P controller on the live camera |
| VERSION | The labs version, 0.9.0 |
| LICENSE | GPLv3 |
| README.md | This file |

## Installation

The driver's setup clones the `main` branch of this repository and copies it over `~/jupyter_ws/neoracer-os/labs`. `racecar update` does the same. A file a student added under a new name is kept. A shipped file a student edited in place is replaced by the new copy, so copy a lab to a new name before you change it. A file removed from this repository stays on the car.

## Running a lab

1. Open a terminal on the car. Programs that open a window need the desktop, not plain SSH.
2. Go to the lab's folder and run it.

   ```sh
   cd ~/jupyter_ws/neoracer-os/labs/lab_a
   python3 lab_a.py
   ```

3. The program starts as soon as it runs. Press Ctrl+C to stop it.

The car follows a program only while the handset's mode switch is down, or while the handset is off. With the switch up, the handset drives the car and the program's drive commands do nothing. Start every program with the switch up, and keep it up, or lift the wheels off the ground, until you are ready. A program that starts with the switch already down can count that as a flip down, and a lab that drives on a flip then moves at once. Center the sticks before you flip the switch up, because the handset drives the car the moment the switch is up.

A speed of 1.0 is the car's full speed, about 6 m/s. Every lab that drives the car, and template.py, has a commented line in start() that sets a lower limit. Uncomment it and pick a speed for your room.

## The hardware the labs see

| Part | Description |
| --- | --- |
| Lidar | 1,440 samples over 360 degrees, 0.25 degrees apart. Index 0 is the front, counting clockwise. The rear 90 degrees read 0 |
| Camera | Color images, 640 by 480, from a USB webcam. No depth image |
| Handset | Joystick.LEFT y is the throttle stick, forward is positive. Joystick.RIGHT x is the steering stick, right is positive. Button.RB is the mode switch down, Button.LB is the mode switch up or in the middle. No other button exists |
| IMU | x forward, y left, z up |
| Encoder | The car's forward speed in m/s |
| Battery | Voltage only |
| Dot matrix | An 8 by 8 panel that shows text |

The depth image, the LED strip, dot matrix pixels and battery current raise NotImplementedError, because the NeoRacer has no such hardware.

## Acknowledgement

These labs were adapted from MIT's outreach labs, racecar-neo-outreach-labs (https://github.com/MITRacecarNeo/racecar-neo-outreach-labs), released under GPLv3. The text and the controls were changed for the NeoRacer, its FlySky handset and the student library 0.9.0.

## License

GPLv3. Copyright Neobotics Foundation Inc.

## Changelog

- 0.9.0: first versioned release, paired with driver 0.9.0 and library v0.9.0-neobotics. The labs describe the NeoRacer and its handset.
