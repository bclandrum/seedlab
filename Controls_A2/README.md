# Localization and Control Assignment 2 – SEED LAB

**Colorado School of Mines**

### Team Members
* Josiah Husmann
* Bradley Landrum
* Cailey Cashion
* Stefan Marshall

## Programs

### `A2_2b_Motor_Encoder_Counts.ino`
Reads the wheel encoders as the wheels are turned by hand and uses the counts to estimate the robot’s position and heading.

### `Assignment2-Part1b.ino`
Runs a DC motor using the motor driver and changes the voltage sign and PWM duty cycle so the signals can be checked with an oscilloscope.

### `Assignment2Part2a.ino.ino`
Uses encoder readings and a proportional controller to control motor speed. Sends time, voltage, and velocity data to MATLAB.

### `Assignment2Part2a.m`
Plots the experimental motor voltage and velocity alongside the simulated results to compare the motor’s response.
