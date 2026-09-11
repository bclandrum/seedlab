import cv2
from cv2 import aruco
import numpy as np
import time
import board
import busio
from time import sleep
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

#LCD setup from github
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
# Initialise the lcd class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows, address=0x20)
# Turn backlight on
lcd.backlight = True


aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

camera = cv2.VideoCapture(0) # Initialize the camera
sleep(.5) # wait for image to stabilize

while True:
    ret,frame = camera.read() # Take an image
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
    corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)
    overlay = cv2.cvtColor(grey,cv2.COLOR_GRAY2RGB) # Convert back to RGB for imshow, as well as for the next step

    if not ids is None:
        ids = ids.flatten()
        overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)
        idText = ", ".join(map(str, ids))
        lcd.clear()
        lcd.message = f"ID: {idText}" 
        sleep(1)

        for (outline, id) in zip(corners, ids):
            markerCorners = outline.reshape((4,2)) 
            overlay = cv2.putText(overlay, str(id),(int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,0,0), 2)
    else:
        lcd.clear()
        lcd.message = "No ArUco\nfound."
        sleep(.5)

    cv2.imshow("overlay",overlay)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
lcd.backlight = False
