# Demo 2 – SEED LAB

**Colorado School of Mines**

### Team Members

* Josiah Husmann
* Bradley Landrum
* Cailey Cashion
* Stefan Marshall

## Programs

### `demo2.py`

Main Pi program for Demo 2. This program:

* Detects ArUco marker using a camera
* Uses solvePnP to get translational vectors between screen center and center of ArUco
* Gets distance from ArUco Marker
* Gets the x-angle and displays it to the LCD screen
* LCD uses threading and time to display the angle every second
* uses masking to find which arrow is next to the ArUco marker
* Sends the angle, distance, and the arrow direction to arduino via I2C


