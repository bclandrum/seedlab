# Josiah Husmann, Bradley Landrum - SEED Lab
# ArUco angle detection
# Program finds the angle from the middle that an ArUco marker is detected from the camera and displays the x-angle on the LCD Screen

import queue
import threading
from random import random
from time import sleep
import cv2
from cv2 import aruco
import numpy as np
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


q = queue.Queue()
def myFunction():
    # ************************
    # Initialize your LCD here
    # ************************
    #LCD setup from github
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
            
camera = cv2.VideoCapture(0) # Initialize the camera
sleep(.5) # wait for image to stabilize

last_lcd_text = ""

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
myThread = threading.Thread(target=myFunction,args=())
myThread.start()   
while True:
    # Do some things...
    putSomething = random()
    # Send it to the thread
    # Only send a new message when the LCD text changes

    # Carry on...
    ret,frame = camera.read() # Take an image
    img_width, img_height = 640, 480
    cx, cy = img_width / 2.0, img_height / 2.0
    fx, fy = img_width,img_height
    cameraMatrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]], dtype=np.float32)
    # Define 3D coordinates of the marker corners in the marker's coordinate space (in meters)
    marker_length = 0.05  # Adjust to your physical marker size
    objPoints = np.array([
        [-marker_length / 2.0,  marker_length / 2.0, 0],
        [ marker_length / 2.0,  marker_length / 2.0, 0],
        [ marker_length / 2.0, -marker_length / 2.0, 0],
        [-marker_length / 2.0, -marker_length / 2.0, 0]
    ], dtype=np.float32)    

    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
    corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)
    overlay = frame.copy() # Keep the original color image for imshow
    if not ids is None:
        ids = ids.flatten()
        overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = (0, 255, 0))
        idText = ", ".join(map(str, ids)) 
        lcdText = f"ID: {idText}" 

        for (outline, id) in zip(corners, ids):
            markerCorners = outline.reshape((4,2)) 
            overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0), 2)

        for i, corner in enumerate(corners):
            # 4 corners of the marker in (x, y) pixel coordinates
            pts = corner[0]

            #calculate center (x, y)
            center_x = np.mean(pts[:, 0])
            center_y = np.mean(pts[:, 1])


            coord_text = f'X: {center_x}, Y: {center_y}'
            # Use solvePnP to get rotation and translation vectors
            ret, rvec, tvec = cv2.solvePnP(objPoints, corner, cameraMatrix, None)
            
            if ret:
                # tvec[0] is X, tvec[2] is Z
                x_angle_rad = np.arctan2(tvec[0][0], tvec[2][0])
                x_angle_deg = np.degrees(x_angle_rad)

                coord_text = f'X: {center_x:.1f}, Y: {center_y:.1f}'
                angle_text = f'Angle: {x_angle_deg:.1f}deg'
                lcdText = f'x-angle: {x_angle_deg:.1f}deg'

                # Overlay angle onto the image
                cv2.putText(overlay, angle_text, (int(center_x - 50), int(center_y + 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 0), 2)
                # Overlay coordinates of center onto the image
                cv2.putText(overlay, coord_text, (int(center_x - 50), int(center_y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0, 255, 0),2,)
    else:
        lcdText = "No ArUco\nfound."

    # Send the new LCD message to the thread only if it changed
    if lcdText != last_lcd_text:
        q.put(lcdText)
        last_lcd_text = lcdText

    cv2.imshow("overlay",overlay)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()

