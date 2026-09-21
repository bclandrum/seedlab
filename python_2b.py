# raspberry pi code for 2b.

import numpy as np
import time
from time import sleep
import board
import busio
from smbus2 import SMBus
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import cv2 # opencv

#LCD setup from github
lcd_columns = 16
lcd_rows = 2
# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)  # uses board.SCL and board.SDA
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows, address=0x20) # Initialise the lcd class
lcd.backlight = True # Turn backlight on
lcd.message = "Capturing..."

while True:
	camera = cv2.VideoCapture(0) # Initialize the camera
	sleep(2) # wait for image to stabilize
	# Get an image from the camera stream, error and quit if not
	ret, image = camera.read()
	if not ret:
		print("Could not capture image from camera!")
		lcd.message = "Cap. Error!"
		quit()
		cv2.imwrite("captured.jpg",image)
	k = cv2.waitKey(1) & 0xFF
	if k == ord('q'):
		break
# threshold for green in hsv
hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
lowGreen  = np.array([35, 40, 40])
highGreen = np.array([90, 255, 255])
# masking out only green
mask=cv2.inRange(hsv,lowGreen,highGreen)
# clean up mask with morphological transformations
kernel=np.ones((5,5),np.uint8)
open=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel,iterations=2) # opening / erode -> dilate
# ^ removes some small stray noise
clean=cv2.morphologyEx(open,cv2.MORPH_CLOSE,kernel,iterations=2) # closing / dilate -> erode
# ^fills in small holes in the image

# find + draw + output contours in the masked image
contours,_ = cv2.findContours(clean,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
contour_result = image.copy()
min_area=300
if contours:
	largeContour = max(contours,key=cv2.contourArea)
	area=cv2.contourArea(largeContour)
else:
	largeContour=None
	area=0

lcd.clear()
if largeContour is not None and area > min_area:
	cv2.drawContours(contour_result,[largeContour],-1,(0,255,0),3)
	x,y,w,h=cv2.boundingRect(largeContour)
	center = (x+w//2,y+h//2)
	cv2.puText(contour_result,"Green shape",(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
	cv2.circle(contour_result,center,4,(0,0,255),-1)
	print("Found the green shape... area=%.0f , center=%s."%(area,center))
	lcd.message = "Green Found!"
else:
	print("Green shape was NOT found.")
	lcd.message ="No Green"

# display + camera breakdown
cv2.imshow("Green Shape",contour_result)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("green_shape_mask.jpg",contour_result)
sleep(3)
camera.release()
cv2.destroyAllWindows()
lcd.backlight = False
lcd.clear()