# seedlab - repository
# Colorado School of Mines - SEED Lab

## Members:
Josiah Husmann, Bradley Landrum, Cailey Cashion, Stefan Marshall

## Project Overview:

This repository serves as the centralized workspace for our SEED Lab coursework. 
Its primary purpose is to organize, share, and version-control the codebase used for our autonomous robotics projects. 
This includes developing OpenCV computer vision pipelines, implementing Raspberry Pi to Arduino communication protocols, 
and managing the overall system architecture.

## Purpose and Organization:

This repository is primarily used to store, organize, and pull down relevant project code into our raspberry pis and arduinos.
The organization of this repository is broken down by individual assignment, ie. Assignment 1, Assignment 2, Mini Project.
Within each assignment folder you can find the specific python and arduino sketch files relevant to a certain assignment.
These files are named by either their assignment name or their purpose, ie. python_1a.py, or MiniProjectPi.py

## Hardware + Relevant Libraries

Raspberry Pi 4 Model B
Arduino UNO R3

numpy (for arrays and computations), time (for sleep and time controls), board (for communicating with raspberry pi), 
busio (for i2C and interconnectivity), smbus2 (for read and write and i2c), 
adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd (for using the lcd display with the raspberri pi),
cv2 (for openCV and computer vision applications).
