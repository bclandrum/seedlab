# Mini Project – SEED LAB

**Colorado School of Mines**

### Team Members

* Josiah Husmann
* Bradley Landrum
* Cailey Cashion
* Stefan Marshall

## Programs

### `MiniProjectPi.py`

Main Raspberry Pi program for the mini project. This program:

* Detects ArUco markers using the camera.
* Determines the quadrant/location of each ArUco marker.
* Displays the desired coordinates on the LCD.
* Transmits the desired coordinates to the Arduino.

### `MiniProjectArduino.ino`

Main Arduino program for the mini project. This program:

* Receives desired coordinates from the Pi
* 

### `threadingTest.py`

Example program used to test LCD threading and demonstrate communication between threads and the LCD display.
