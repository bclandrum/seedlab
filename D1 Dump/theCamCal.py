import numpy as np
import cv2
import json

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points for an 8x8 checkerboard (7x7 internal corners)
# multiplied by 20 to represent the 20mm square dimensions
objp = np.zeros((7*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:7].T.reshape(-1, 2) * 20

# Arrays to store object points and image points
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane

# Start video capture
cap = cv2.VideoCapture(0)

# Force the 640x480 resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Press 's' to capture a calibration pattern.")
print("Press 'c' to finish capturing and start calibration.")

captured_count = 0
gray_shape = None

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    # Display the live feed
    cv2.imshow('Live Camera - Press s to save, c to calculate', frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if gray_shape is None:
            gray_shape = gray.shape[::-1]
        
        # Find the chess board corners
        ret_corners, corners = cv2.findChessboardCorners(gray, (7, 7), None)

        # If found, add object points, image points (after refining them)
        if ret_corners:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            
            # Draw and display the corners to confirm capture[cite: 2]
            cv2.drawChessboardCorners(frame, (7, 7), corners2, ret_corners)
            cv2.imshow('Live Camera - Press s to save, c to calculate', frame)
            cv2.waitKey(500) # Pause for 500ms to show the drawn corners[cite: 2]
            
            captured_count += 1
            print(f"Captured {captured_count} images successfully!")
        else:
            print("Checkerboard not found in this frame. Try adjusting the angle or lighting.")

    elif key == ord('c'):
        print("Finished image capture. Starting calibration...")
        break

cap.release()
cv2.destroyAllWindows()

if captured_count < 10:
    print("Note: For better results, you need at least 10 test patterns[cite: 2].")

if len(objpoints) > 0:
    # Calibrate the camera to get the camera matrix (mtx) and distortion coefficients (dist)[cite: 3]
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_shape, None, None)

    # Convert NumPy arrays to standard Python lists for JSON serialization
    camera_matrix_data = mtx.tolist()
    distortion_data = dist.tolist()

    # Write the camera matrix to a JSON file
    with open('cameraMatrix.json', 'w') as f:
        json.dump(camera_matrix_data, f, indent=4)

    # Write the distortion coefficients to a JSON file
    with open('distortionCoef.json', 'w') as f:
        json.dump(distortion_data, f, indent=4)

    print("Calibration complete. Data saved to 'camera_matrix.json' and 'distortion_coefficients.json'.")
else:
    print("No images were captured. Calibration failed.")
