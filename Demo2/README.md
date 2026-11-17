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

### `Demo2Controls.ino`

# Files
- Demo2Controls.ino -- hardware settings, encoder measurements, wheel control, and navigation
- VisionI2C.h -- receives and validates Pi messages, converts camera measurements, and provides controller status

Make sure to keep both files in the same Arduino sketch folder

# Hardware and wiring
The sketch uses an Arduino Uno, two encoder-equipped motors, a motor driver, and a Raspberry Pi running vision code.

# Navigation
State machine:
- Searching (0): Rotates until a fresh marker observation is available, then starts approaching
- Aligning (1): Stops and waits after losing the marker during approach. Resumes approach when vision returns, or returns to searching after sustained loss.
- Approaching (2): Drives at a constant commanded forward speed while correcting bearing. At the stopping threshold, cuts motor power and confirms arrival
- Final_Turn (3): Uses encoder position feedback to execute the calibrated turn
- Done (4): Disables the motors until rest. Also used for manual stops and timeouts

The robot starts approaching as soon as it detects the marker. It does not wait to become perfedcting centered.

Arrival is latched once the measured distance reaches the stopping threshold. The robot then coasts to a stop and waits for low wheel speeds for 25 controlled samples, approximately 250 ms.

# I2C Communication
The Pi and I2C controller and the Arduino is peripheral address 8. The agreed Python payload is packed as <ffB. 

For the agreed format, the Pi sends nine payload bytes. The leading register byte makes ten bytes at the Arduino receiver.

Invalid lengths, direction values, nonfinite values, distances outside of 0-10m, and out-of-range angles are rejected. Messages contain no frame sequence or explicit no-detection flag. After 500 ms without an accepted observation, the Arduino stops forward motion. 

# Setup and Testing

1. Open Demo2Controls.ino in Arduino IDE with VisionI2C.h. The sketch uses the built-in Wire library and manual encoder interrupts.
2. Select the Arduino Uno and upload the sketch
3. For stationary checks, set COMMUNICATION_TEST_ONLY = true. Verify recieved distance, bearing, direction, and packet counters using Serial Monitor at 115200 baud.
4. Set the trial type and motion settings for the run. Set COMMUNICATION_TEST_ONLY = false and upload again.
5. Prepare the Pi vision program and robot power, then reset the Arduino to start. The current zero-delay setting starts search immediately.
6. Test approach-only operation, then both final-turn directions. Measure the final gap from the midpoint between the wheels and check the actual turn angles.

# Tuning Notes

- Increase APPROACH_SPEED_M_S for faster forward travel, then remeasure the stopping gap because the robot coasts after power is cut
- Increase SEARCH_YAW_RAD_S for faster searching and verify that vision still detects the marker reliably
- Change TURN_SPEED_RAD_S to adjust the final-turn wheel-speed cap
- Change STOP_DISTANCE_M to adjust where stopping begins. Keep it at or below ARRIVAL_LIMIT_M



