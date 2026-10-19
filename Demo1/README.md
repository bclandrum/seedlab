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

### `Demo1.ino`

Main Arduino program for Demo1. This program:

* TBD


### `camcal.py`

This program was used to get the calibration values for the camera.

