# Josiah Husmann, Bradley Landrum - SEED Lab
# ArUco angle detection & distance measurement
# Program finds the angle and distance of an ArUco marker from the camera and displays it
#
# Fixes in this version:
#   1. Distortion is only corrected once (frame is undistorted, so solvePnP gets None for distortion)
#   2. Capture resolution is forced to match the calibration resolution (and the matrix is
#      rescaled with a warning if the camera refuses)
#   3. Marker object points are centered, so tvec points to the marker CENTER, not a corner
#   4. Uses SOLVEPNP_IPPE_SQUARE, which is designed for square markers and avoids pose flipping
#   5. Autofocus disabled (a changing focus changes the focal length and breaks calibration)
#   6. Undistortion maps / new camera matrix computed once instead of every frame
#   7. LCD text fits the 16x2 display and shows the closest marker's angle and distance

import json
import os
import queue
import threading
from time import sleep, time

import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import busio
import cv2
from cv2 import aruco
import numpy as np

# --- Configuration --- #
FRAME_SIZE = (640, 480)     # MUST match the resolution used in camcal2.py
MARKER_LENGTH = 0.05        # meters. Measure ONLY the outer black border of the printed marker!
ANGLE_SIGN = 1              # +1: marker to the right of the camera reads positive. Use -1 to flip.
LCD_UPDATE_INTERVAL = 1.0   # seconds between LCD updates

q = queue.Queue()


def lcd_worker():
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


# --- Camera setup --- #
camera = cv2.VideoCapture(0)
if not camera.isOpened():
  raise SystemExit("Error: could not open camera.")

camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SIZE[0])
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])
camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # ignored by cameras that don't support it
sleep(0.5)  # wait for image to stabilize

# --- Load calibration --- #
with open("cameraMatrix.json", "r") as file1:
  initCameraMatrix = np.array(json.load(file1), dtype=np.float64)
with open("distortionCoef.json", "r") as file2:
  distortionCoefs = np.array(json.load(file2), dtype=np.float64)

# Resolution the calibration was done at (saved by the updated camcal2.py)
calibSize = FRAME_SIZE
if os.path.exists("calibrationSize.json"):
  with open("calibrationSize.json", "r") as file3:
    calibSize = tuple(json.load(file3))

# Grab one frame to find the real capture size
ret, frame = camera.read()
if not ret:
  raise SystemExit("Error: could not read a frame from the camera.")
h, w = frame.shape[:2]
print(f"Camera resolution: {w}x{h} (calibrated at {calibSize[0]}x{calibSize[1]})")

if (w, h) != tuple(calibSize):
  # Rescale the intrinsics. Only accurate if the aspect ratio is the same and the
  # camera scales (not crops) between resolutions. Recalibrating at this size is better.
  print("WARNING: capture resolution does not match calibration resolution! "
        "Rescaling camera matrix, but you should recalibrate at this resolution.")
  sx = w / calibSize[0]
  sy = h / calibSize[1]
  initCameraMatrix[0, 0] *= sx
  initCameraMatrix[0, 2] *= sx
  initCameraMatrix[1, 1] *= sy
  initCameraMatrix[1, 2] *= sy

# Compute the undistorted camera matrix and remap tables ONCE
cameraMatrix, roi = cv2.getOptimalNewCameraMatrix(
    initCameraMatrix, distortionCoefs, (w, h), 1, (w, h)
)
map1, map2 = cv2.initUndistortRectifyMap(
    initCameraMatrix, distortionCoefs, None, cameraMatrix, (w, h), cv2.CV_16SC2
)

# 3D marker corners, CENTERED on the marker, in the order ArUco returns them
# (top-left, top-right, bottom-right, bottom-left). Required by SOLVEPNP_IPPE_SQUARE.
half = MARKER_LENGTH / 2.0
objPoints = np.array(
    [
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ],
    dtype=np.float32,
)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

last_lcd_text = ""
last_lcd_update_time = 0

# daemon=True so the program can exit cleanly when 'q' is pressed
myThread = threading.Thread(target=lcd_worker, args=(), daemon=True)
myThread.start()

while True:
  ret, frame = camera.read()  # Take an image
  if not ret:
    break

  # Remove lens distortion. From here on the image is distortion-free,
  # so we pass None (no distortion) to solvePnP below.
  newFrame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

  grey = cv2.cvtColor(newFrame, cv2.COLOR_BGR2GRAY)  # greyscale for ArUco detection
  corners, ids, rejected = aruco.detectMarkers(grey, aruco_dict)
  overlay = newFrame.copy()  # Keep the color image for imshow

  if ids is not None:
    ids = ids.flatten()
    overlay = aruco.drawDetectedMarkers(overlay, corners, borderColor=(0, 255, 0))
    lcdText = "Marker found\nNo pose"
    closest_distance = None

    for corner, marker_id in zip(corners, ids):
      pts = corner.reshape((4, 2))

      # ID label above the marker
      cv2.putText(
          overlay,
          str(marker_id),
          (int(pts[0, 0]), int(pts[0, 1]) - 15),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5,
          (255, 0, 0),
          2,
      )

      # Pose: distortion is None because newFrame is already undistorted
      ret_pnp, rvec, tvec = cv2.solvePnP(
          objPoints, pts, cameraMatrix, None, flags=cv2.SOLVEPNP_IPPE_SQUARE
      )
      if not ret_pnp:
        continue

      # tvec = (X right, Y down, Z forward) from camera to marker CENTER, in meters
      x, y, z = tvec.flatten()
      x_angle_deg = ANGLE_SIGN * np.degrees(np.arctan2(x, z))

      # Euclidean distance from camera to marker center (meters)
      distance = np.linalg.norm(tvec)
      # Alternatively, forward depth only:
      # distance = z

      # Draw the marker's axes - a good visual check that the pose is sane
      cv2.drawFrameAxes(overlay, cameraMatrix, None, rvec, tvec, MARKER_LENGTH / 2)

      center_x = int(np.mean(pts[:, 0]))
      center_y = int(np.mean(pts[:, 1]))
      cv2.putText(
          overlay,
          f"Angle: {x_angle_deg:.1f}deg",
          (center_x - 50, center_y + 10),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5,
          (100, 255, 0),
          2,
      )
      cv2.putText(
          overlay,
          f"Dist: {distance:.2f}m",
          (center_x - 50, center_y + 30),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5,
          (100, 255, 0),
          2,
      )

      # LCD shows the closest marker. Each line stays within 16 characters.
      if closest_distance is None or distance < closest_distance:
        closest_distance = distance
        lcdText = f"ID{marker_id} Ang:{x_angle_deg:.1f}\nDist: {distance:.2f}m"
  else:
    lcdText = "No ArUco\nfound."

  # Rate-limit LCD updates to at most once per LCD_UPDATE_INTERVAL
  current_time = time()
  if lcdText != last_lcd_text and (
      current_time - last_lcd_update_time >= LCD_UPDATE_INTERVAL
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
