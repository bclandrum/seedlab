# Josiah Husmann, Bradley Landrum - SEED Lab
# Mini Project Pi Code
# This program identifies aruco markers and updates the LCD using threading with the goal coordinates and sends the quadrant to the arduino

import queue
import threading
from random import random
from time import sleep
import cv2
from cv2 import aruco
import numpy as np
import board
import busio
from smbus2 import SMBus
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


q = queue.Queue()
#Arduino init
ARDUINO_ADDR = 8
bus = SMBus(1)
value = 0
def goal_loc(cx,cy,frame_width=640,frame_height=480):
    x_axis= frame_height//2 # horizontal threshold (divides vertical in half across the middle)
    y_axis= frame_width//2 # vertical threshold (divides horizontal in half across the middle)

    if cx >= y_axis and cy <= x_axis: #north east quadrant
        left,right = 0,0
        value = 1
        lcdNew = f"Goal Position:\n{left},{right}"
    elif cx <= y_axis and cy <= x_axis: #north west quadrant
        left,right = 0,1
        value = 2
        lcdNew = f"Goal Position:\n{left},{right}"
    elif cx <= y_axis and cy >= x_axis: #south west quadrant
        left,right = 1,1
        value = 3
        lcdNew = f"Goal Position:\n{left},{right}"
    elif cx >= y_axis and cy >= x_axis: #south east quadrant
        left,right = 1,0
        value = 4
        lcdNew = f"Goal Position:\n{left},{right}"

    return left, right, lcdNew, value

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
        # ******************************
        # Write new data to the LCD here
        # ******************************
            
camera = cv2.VideoCapture(0) # Initialize the camera
sleep(.5) # wait for image to stabilize

last_lcd_text = ""
lcdText = ""
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

            # calculate center (x, y)
            center_x = np.mean(pts[:, 0])
            center_y = np.mean(pts[:, 1])

            # gets the goal coordinate values
            x_goal,y_goal,lcdText,value = goal_loc(center_x, center_y)

            coord_text = f'X: {center_x}, Y: {center_y}'
            cv2.line(overlay, (320,0), (320,480), (0, 0, 255), 2)
            cv2.line(overlay, (0,240), (640,240), (0, 0, 255), 2)   
            # Overlay coordinates of center onto the image
            # cv2.putText(overlay, coord_text, (int(center_x - 50), int(center_y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0, 255, 0),2,)
    else:
        lcdText = "No ArUco\nfound."

    # Send the new LCD message to the thread only if it changed
    if lcdText != last_lcd_text:
        q.put(lcdText)
        last_lcd_text = lcdText
    bus.write_byte(ARDUINO_ADDR, value) # starts receiving on Arduino
    response = bus.read_byte(ARDUINO_ADDR) # starts reading on Arduino
    cv2.imshow("overlay",overlay)
    #if user enters q then quit
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
