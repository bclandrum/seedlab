while True:
  ret, frame = camera.read()  # Take an image
  if not ret:
    break

  grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Greyscale for detection
  corners, ids, rejected = aruco.detectMarkers(grey, aruco_dict)
  overlay = frame.copy()  # Keep original color frame for display

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
      pts = corner[0]
      center_x = np.mean(pts[:, 0])
      center_y = np.mean(pts[:, 1])

      # Use original initCameraMatrix and distortionCoefs directly on raw frame
      ret_pnp, rvec, tvec = cv2.solvePnP(
          objPoints, corner[0], initCameraMatrix, distortionCoefs
      )

      if ret_pnp:
        x_angle_rad = np.arctan2(tvec[0][0], tvec[2][0])
        x_angle_deg = np.degrees(x_angle_rad)

        # Calculate Euclidean distance from camera to marker (in meters)
        distance = np.linalg.norm(tvec)

        angle_text = f"Angle: {x_angle_deg:.1f}deg"
        dist_text = f"Dist: {distance:.2f}m"

        lcdText = f" Angle: {x_angle_deg:.1f} degrees"

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
