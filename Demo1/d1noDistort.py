# Josiah Husmann, Bradley Landrum - SEED Lab
# ArUco angle detection & distance measurement
# Program finds the angle and distance of an ArUco marker from the camera and displays it

import json
import queue
import threading
from random import random
from time import sleep, time
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import busio
import cv2
from cv2 import aruco
import numpy as np

q = queue.Queue()


def myFunction():
  # LCD setup from github
  lcd_columns = 16
  lcd_rows = 2

  # Initialise I2C bus.
  i2c = busio.I2C(board.SCL, board.SDA)  # uses board.SCL and board.SDA
  # Initialise the lcd class
  lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows, address=0x20)
  # Turn backlight on
  lcd.backlight = True

  while True:
    # Wait for a new message
    lcdText = q.get()

    # Clear the old message
    lcd.clear()

    # Write the new data to the LCD
    lcd.message = lcdText


camera = cv2.VideoCapture(0)  # Initialize the camera
sleep(0.5)  # wait for image to stabilize
with open("cameraMatrix.json", "r") as file1:
  initCameraMatrix = np.array(json.load(file1))
with open("distortionCoef.json", "r") as file2:
  distortionCoefs = np.array(json.load(file2))

last_lcd_text = ""
last_lcd_update_time = 0
lcd_update_interval = 1.0  # Update LCD once every second

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
myThread = threading.Thread(target=myFunction, args=())
myThread.start()

while True:
  ret, frame = camera.read()  # Take an image
  if not ret:
    break
  h, w = frame.shape[:2]

  # Unpack tuple properly so cameraMatrix is a valid NumPy array
  cameraMatrix, roi = cv2.getOptimalNewCameraMatrix(
      initCameraMatrix, distortionCoefs, (w, h), 1, (w, h)
  )


  # Define 3D coordinates of the marker corners in the marker's coordinate space (in meters)
  marker_length = 0.05  # <--- Make sure this matches your exact physical marker size in meters!
  objPoints = np.array(
      [
          [0.0, 0.0, 0.0],
          [marker_length, 0.0, 0.0],
          [marker_length, marker_length, 0.0],
          [0.0, marker_length, 0.0],
      ],
      dtype=np.float32,
  )

  grey = cv2.cvtColor(
      frame, cv2.COLOR_BGR2GRAY
  )  # Make the image greyscale for ArUco detection
  corners, ids, rejected = aruco.detectMarkers(grey, aruco_dict)
  overlay = frame.copy()  # Keep the original color image for imshow

  if ids is not None:
    ids = ids.flatten()
    overlay = aruco.drawDetectedMarkers(overlay, corners, borderColor=(0, 255, 0))
    idText = ", ".join(map(str, ids))
    lcdText = f"ID: {idText}"

    for (outline, id) in zip(corners, ids):
      markerCorners = outline.reshape((4, 2))
      overlay = cv2.putText(
          overlay,
          str(id),
          (int(markerCorners[0, 0]), int(markerCorners[0, 1]) - 15),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5,
          (255, 0, 0),
          2,
      )

    for i, corner in enumerate(corners):
      # 4 corners of the marker in (x, y) pixel coordinates
      pts = corner[0]

      # calculate center (x, y)
      center_x = np.mean(pts[:, 0])
      center_y = np.mean(pts[:, 1])

      # Pass corner[0] instead of corner so solvePnP receives the correct (4, 2) shape
      ret_pnp, rvec, tvec = cv2.solvePnP(
          objPoints, corner[0], cameraMatrix, distortionCoefs
      )

      if ret_pnp:
        # tvec[0] is X, tvec[1] is Y, tvec[2] is Z
        x_angle_rad = np.arctan2(tvec[0][0], tvec[2][0])
        x_angle_deg = np.degrees(x_angle_rad)

        # Calculate Euclidean distance from camera to marker (in meters)
        distance = np.linalg.norm(tvec)
        # Alternatively, if you only want the forward depth (Z-axis):
        # distance = tvec[2][0]



        angle_text = f"Angle: {x_angle_deg:.1f}deg"
        dist_text = f"Dist: {distance:.2f}m"

        # Format text for LCD (max 16 characters per line)
        lcdText = f" Angle: {x_angle_deg:.1f} degrees"

        # Overlay angle and distance onto the image
        cv2.putText(
            overlay,
            angle_text,
            (int(center_x - 50), int(center_y + 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (100, 255, 0),
            2,
        )
        cv2.putText(
            overlay,
            dist_text,
            (int(center_x - 50), int(center_y + 30)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (100, 255, 0),
            2,
        )
  else:
    lcdText = "No ArUco\nfound."

  # Rate-limit LCD updates to occur at most once every 1 second
  current_time = time()
  if lcdText != last_lcd_text and (
      current_time - last_lcd_update_time >= lcd_update_interval
  ):
    q.put(lcdText)
    last_lcd_text = lcdText
    last_lcd_update_time = current_time

  cv2.imshow("overlay", overlay)
  k = cv2.waitKey(1) & 0xFF
  if k == ord("q"):
    break

camera.release()
cv2.destroyAllWindows()
