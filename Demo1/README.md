# Demo 1 – SEED LAB

**Colorado School of Mines**

### Team Members

* Josiah Husmann
* Bradley Landrum
* Cailey Cashion
* Stefan Marshall

## Programs

### `Demo1.py`

Main Pi program for Demo1. This program:

* Detects ArUco marker using a camera
* Uses solvePnP to get translational vectors between screen center and center of ArUco
* Gets the x-angle and displays it to the LCD screen
* LCD uses threading and time to display the angle every second



### `camcal.py`

This program was used to get the calibration values for the camera.

### `Demo1_Independent_Wheel_PI.ino`

Each wheel uses encoder feedback and two control loops:
- **Position P controller:** converts wheel-position error into a desired wheel speed
- **Speed PI controller:** converts speed error into a motor-voltage command

Both integral gains are currently zero, so the speed controllers operate proportional controllers. 

Movement sequence is 'WAITING -> TURNING -> DRIVING -> STOPPED' 
A zero turn angle skips turning, and a zero distance allows a turn-only run

## Running the Sketch

1. Open the program in Arduino IDE and select the board and port
2. Set the movement commands near the top of the sketch:

  '''cpp
  const float DISTANCE_FT = 1.0f;
  const float ANGLE_DEG = 180.0f;
  const unsigned long START_DELAY_MS = 0;
  '''

  Distance is in feet. Positive angles turn left, negative right.

3. Upload the sketch.

## Tuning

- 'KP_POS' controls the position-to-speed response
- 'KP_VEL_L' and 'KP_VEL_R' adjust wheel speed controllers

Movement finishes when both wheel-position errors are within the specified tolerance


