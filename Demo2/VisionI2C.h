// Demo 2 Raspberry Pi -> Arduino I2C compatibility layer.
// Direction codes: 0=left, 1=right, 2=stop.
// Supported layouts (little-endian IEEE-754 float32):
//   [distance f32][angle f32][direction u8]                 = 9 bytes
//   [register 0][distance f32][angle f32][direction u8]    = 10 bytes
//   [distance f32][angle f32][direction i32]               = 12 bytes
//   [register 0][distance f32][angle f32][direction i32]  = 13 bytes
// In the receive callback, copy bytes ONLY; do decoding in the loop.
#pragma once

void receiveVision(int byteCount) {
  byte buffer[VISION_PACKET_SIZE];
  byte n = 0;
  bool overflow = false;
  while (Wire.available()) {
    byte v = Wire.read();
    if (n < VISION_PACKET_SIZE) buffer[n++] = v;
    else overflow = true;
  }
  visionPacketsSeen++;
  if (overflow || byteCount != n ||
      !(n == 9 || n == 10 || n == 12 || n == 13)) {
    visionPacketsRejected++;
    return;
  }
  for (byte i = 0; i < n; ++i) pendingVision[i] = buffer[i];
  pendingVisionLength = n;
  pendingVisionMs = millis();
  visionPending = true;
}

void requestStatus() {
  byte reply[4];
  for (byte i = 0; i < 4; ++i) reply[i] = statusReply[i];
  Wire.write(reply, sizeof(reply));
}

bool visionFresh(unsigned long now) {
  return visionReceived && now - lastVisionMs <= VISION_TIMEOUT_MS;
}

void updateVision(unsigned long now) {
  byte packet[VISION_PACKET_SIZE];
  byte n = 0;
  bool available;
  unsigned long receivedMs = 0;
  newVisionThisTick = false;
  noInterrupts();
  available = visionPending;
  if (available) {
    n = pendingVisionLength;
    for (byte i = 0; i < n; ++i) packet[i] = pendingVision[i];
    receivedMs = pendingVisionMs;
    visionPending = false;
  }
  interrupts();
  if (!available || now - receivedMs > VISION_TIMEOUT_MS) return;

  const bool hasRegister = (n == 10 || n == 13);
  const byte offset = hasRegister ? 1 : 0;
  if (hasRegister && packet[0] != 0) {
    visionPacketsInvalid++;
    return;
  }
  // The float fields are always 4 bytes each, in this order.
  float distance, angle;
  memcpy(&distance, packet + offset, sizeof(float));
  memcpy(&angle, packet + offset + 4, sizeof(float));

  // Numeric direction may be a 1-byte B or a 4-byte i/I.
  byte arrow = DIR_UNKNOWN;
  if (n == 9 || n == 10) {
    arrow = packet[offset + 8];
  } else {
    // Little-endian signed/unsigned int32: values 0,1,2 only.
    if (packet[offset + 9] != 0 || packet[offset + 10] != 0 ||
        packet[offset + 11] != 0) {
      visionPacketsInvalid++;
      return;
    }
    arrow = packet[offset + 8];
  }
  if (!isfinite(distance) || !isfinite(angle) ||
      distance <= 0 || distance > 10 ||
      angle < -180 || angle > 180 || arrow > DIR_STOP) {
    visionPacketsInvalid++;
    return;
  }

  float planarSquared = distance * distance -
      CAMERA_MARKER_HEIGHT_DIFFERENCE_M * CAMERA_MARKER_HEIGHT_DIFFERENCE_M;
  if (planarSquared <= 0) { visionPacketsInvalid++; return; }
  float planarRange = sqrtf(planarSquared);
  float cameraAngle = angle * PI / 180.0f;
  float forward = planarRange * cosf(cameraAngle) + CAMERA_FORWARD_M;
  // Pi bearing is negative for a marker on the robot's right.
  float left = planarRange * sinf(cameraAngle) + CAMERA_LEFT_M;
  if (forward <= 0) { visionPacketsInvalid++; return; }

  newVisionThisTick = true;
  visionSequence++;
  beaconDetected = true;
  beaconDistanceM = sqrtf(forward * forward + left * left);
  beaconAngleDeg = atan2f(left, forward) * 180.0f / PI;
  arrowDirection = arrow;
  lastVisionMs = receivedMs;
  visionReceived = true;
}

void publishStatus(unsigned long now) {
  byte flags = (visionReceived ? 1 : 0) | (visionFresh(now) ? 2 : 0) |
               (visionFresh(now) && beaconDetected ? 4 : 0) |
               (!COMMUNICATION_TEST_ONLY ? 8 : 0) | (runStarted ? 16 : 0) |
               (runAborted ? 32 : 0) | (atBeacon ? 64 : 0);
  noInterrupts();
  statusReply[0] = STATUS_VERSION;
  statusReply[1] = visionSequence;
  statusReply[2] = (byte)state;
  statusReply[3] = flags;
  interrupts();
}
