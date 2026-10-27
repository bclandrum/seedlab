import cv2
import cv2.aruco as aruco
import numpy as np
import json
import threading
import queue
import time
import board
import busio
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
        # Wait for a new message to display
        msg = lcd_queue.get()
        if msg == "QUIT":
            lcd.clear()
            break
        lcd.clear()
        lcd.message = msg

def main():
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
    
    # ArUco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    
    # Define physical marker size (meters)
    marker_length = 0.05 
    half_l = marker_length / 2.0
    
    # 3D points centered at (0,0,0). Standard ArUco order: TL, TR, BR, BL
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
            
            # Process the first detected marker for the LCD
            marker_corners = corners[0][0]
            
            # Use solvePnP to get rotation and translation vectors
            success, rvec, tvec = cv2.solvePnP(
                obj_points, marker_corners, new_camera_matrix, dist_coeffs
            )
            
            if success:
                # X and Z coordinates from the translation vector
                x = tvec[0][0]
                z = tvec[2][0]
                
                # Calculate distance (Euclidean magnitude of tvec)
                distance = np.linalg.norm(tvec)
                
                # Calculate bearing angle (horizontal angle from camera center to marker)
                angle_rad = np.arctan2(x, z)
                angle_deg = np.degrees(angle_rad)
                
                # Format text strings
                dist_str = f"{distance:.2f}m"
                angle_str = f"{angle_deg:.1f} deg"
                
                lcd_msg = f"ID: {ids[0][0]}\nAng:{angle_str} D:{dist_str}"
                
                # Draw text near the marker center
                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))
                cv2.putText(undistorted_frame, f"Dist: {dist_str}", (center_x - 40, center_y + 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(undistorted_frame, f"Ang: {angle_str}", (center_x - 40, center_y + 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Update LCD strictly once per second
        current_time = time.time()
        if current_time - last_lcd_update >= 1.0:
            lcd_queue.put(lcd_msg)
            last_lcd_update = current_time

        cv2.imshow("ArUco Angle Detection", undistorted_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    lcd_queue.put("QUIT")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
