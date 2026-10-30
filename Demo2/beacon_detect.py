import cv2
import cv2.aruco as aruco
import numpy as np
import json
import threading
import queue
import time
import board
import busio
import struct
from smbus2 import SMBus
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Queue for passing LCD messages safely between threads
lcd_queue = queue.Queue()

def lcd_worker():
    """Background thread to handle LCD updates without blocking the camera feed."""
    i2c = busio.I2C(board.SCL, board.SDA)
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, 16, 2, address=0x20)
    lcd.backlight = True
    lcd.clear()
    
    while True:
        msg = lcd_queue.get()
        if msg == "QUIT":
            lcd.clear()
            break
        lcd.clear()
        lcd.message = msg


# Start the LCD threading
lcd_thread = threading.Thread(target=lcd_worker, daemon=True)
lcd_thread.start()

# Load camera calibration data
with open("cameraMatrix.json", "r") as f:
    camera_matrix = np.array(json.load(f))
with open("distortionCoef.json", "r") as f:
    dist_coeffs = np.array(json.load(f))

# Initialize camera
cap = cv2.VideoCapture(0)
time.sleep(0.5) # wait for image to stabilize


aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

marker_length = 0.0508 
half_l = marker_length / 2.0

# 3D points centered at (0,0,0)
obj_points = np.array([
    [-half_l,  half_l, 0],
    [ half_l,  half_l, 0],
    [ half_l, -half_l, 0],
    [-half_l, -half_l, 0]
], dtype=np.float32)

last_lcd_update = 0.0

while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    h, w = frame.shape[:2]

    # Calculate optimal camera matrix and undistort the frame
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    undistorted_frame = cv2.undistort(
        frame, camera_matrix, dist_coeffs, None, new_camera_matrix
    )

    gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict)
    
    lcd_msg = "No Marker\nFound"

    if ids is not None and len(ids) > 0:
        aruco.drawDetectedMarkers(undistorted_frame, corners, ids)
        
        # Process the first detected marker
        marker_corners = corners[0][0]
        marker_id = ids[0][0]
        
        # Use solvePnP to get rotation and translation vectors
        success, rvec, tvec = cv2.solvePnP(
            obj_points, marker_corners, new_camera_matrix, dist_coeffs
        )
        
        if success:
            x, z = tvec[0][0], tvec[2][0]
            distance = np.linalg.norm(tvec)
            angle_deg = np.degrees(np.arctan2(x, z))

            
            # Find the bounding box of the marker in 2D image space
            x_coords = marker_corners[:, 0]
            y_coords = marker_corners[:, 1]
            x_min, x_max = int(np.min(x_coords)), int(np.max(x_coords))
            y_min, y_max = int(np.min(y_coords)), int(np.max(y_coords))
            m_width = x_max - x_min
            
            # Define ROIs to the left and right of the marker
            left_roi = undistorted_frame[y_min:y_max, max(0, x_min - m_width) : max(0, x_min)]
            right_roi = undistorted_frame[y_min:y_max, min(w, x_max) : min(w, x_max + m_width)]
            
            # Convert ROIs to HSV
            hsv_left = cv2.cvtColor(left_roi, cv2.COLOR_BGR2HSV) if left_roi.size > 0 else np.zeros((1,1,3), dtype=np.uint8)
            hsv_right = cv2.cvtColor(right_roi, cv2.COLOR_BGR2HSV) if right_roi.size > 0 else np.zeros((1,1,3), dtype=np.uint8)
            
            # Masks for Green (Left arrow) and Red (Right arrow)[cite: 7]
            mask_green = cv2.inRange(hsv_left, np.array([40, 50, 50]), np.array([90, 255, 255]))
            
            mask_red1 = cv2.inRange(hsv_right, np.array([0, 50, 50]), np.array([10, 255, 255]))
            mask_red2 = cv2.inRange(hsv_right, np.array([170, 50, 50]), np.array([180, 255, 255]))
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            
            green_pixels = cv2.countNonZero(mask_green)
            red_pixels = cv2.countNonZero(mask_red)
            
            # True if Green (Left) is dominant, False if Red (Right)[cite: 7]
            is_left = bool(green_pixels > red_pixels)
            dir_str = "Left" if is_left else "Right"
            
            # Format strings for LCD and Display
            dist_str = f"{distance:.2f}m"
            angle_str = f"{angle_deg:.1f} deg"
            lcd_msg = f"D:{dist_str} A:{angle_str}\nDir: {dir_str}"
            
            # Draw text near the marker center
            center_x, center_y = int(np.mean(x_coords)), int(np.mean(y_coords))
            cv2.putText(undistorted_frame, f"Dist: {dist_str}", (center_x - 40, center_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(undistorted_frame, f"Ang: {angle_str}", (center_x - 40, center_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(undistorted_frame, f"Dir: {dir_str}", (center_x - 40, center_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)


    current_time = time.time()
    if current_time - last_lcd_update >= 1.5:
        lcd_queue.put(lcd_msg)
        last_lcd_update = current_time

    cv2.imshow("ArUco Angle & Beacon Detection", undistorted_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
lcd_queue.put("QUIT")
cap.release()
cv2.destroyAllWindows()
