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
* Converts the quadrant into a desired position of 0 or π radians for each wheel.
* Uses encoder interrupts to track wheel rotation at 1600 counts per revolution.
* Calculates each wheel’s position and speed from the encoder counts.
* Uses a PI position controller to calculate desired speed and a proportional speed controller to calculate the motor voltage command.
* Controls each motor’s direction and PWM duty cycle.
* Stops each wheel within approximately 2° of its target and resumes correction if displaced more than approximately 4°.
* Prints target positions, measured positions, voltage commands, and control states to the Serial Monitor.

#### Startup Procedure

1. Connect the Arduino by USB with motor power disconnected.
2. Open the Serial Monitor at **115200 baud**.
3. Align both wheels with their zero marks facing upward.
4. Send `z` to establish the encoder zero positions.
5. Enable motor power and send `p` to enable Pi commands.
6. Have the Pi send a fresh quadrant number to begin positioning.

Send `s` to stop both motors and disable Pi control. Send `p` again
to accept a fresh Pi command. Zeroing with `z` is permitted only while stopped.

### `threadingTest.py`

Example program used to test LCD threading and demonstrate communication between threads and the LCD display.
