// Assignment 2b: Localization and Control
// Cailey Cashion - 09/21/26
// Estimates robots position and orientation from hand-turning the wheels

// encoder A pins trigger interrupts
// encoder B pins determine rotationa and direction
const byte LEFT_A  = 2;
const byte LEFT_B  = 5;
const byte RIGHT_A = 3;
const byte RIGHT_B = 6;

// physical measurements
const float COUNTS_PER_REV = 1600.0; // one complete wheel rotation
const float WHEEL_RADIUS = 0.06985;  // meters
const float WHEEL_SPACING = 0.3175; // meters, center-to-center

// Each wheel must count positively when moving the robot forward.
const int LEFT_SIGN = -1; // changed to -1 because the movement was opposite of what it should be
const int RIGHT_SIGN = 1;

// interrupt functions update these encoder counts
// volatile: can change outside of the main loop
volatile long leftCount = 0;
volatile long rightCount = 0;

// counts are saved from the earlier localization update
long previousLeft = 0;
long previousRight = 0;

// estimated position and orientation (relative to starting pos.)
float x = 0.0; // meters
float y = 0.0; // meters
float heading = 0.0; // radians

// timing variables (milliseconds)
unsigned long startTime;
unsigned long lastUpdate;
unsigned long lastPrint;

// left encoder reading (when A signal changes)
// Subtract or add one count: compare A and B
void readLeftEncoder() {
  if (digitalRead(LEFT_A) == digitalRead(LEFT_B)) {
    leftCount++; // reads the 1600 count
  } else {
    leftCount--; // same here
  }
}

// right encoder reading
void readRightEncoder() {
  if (digitalRead(RIGHT_A) == digitalRead(RIGHT_B)) {
    rightCount++; // reads the 1600 count
  } else {
    rightCount--; // same here
  }
}

void setup() {
  // configures encoder signals as inputs with internal pull-up resistors
  pinMode(LEFT_A, INPUT_PULLUP);
  pinMode(LEFT_B, INPUT_PULLUP);
  pinMode(RIGHT_A, INPUT_PULLUP);
  pinMode(RIGHT_B, INPUT_PULLUP);

  // Disable motor driver because the wheels are turned by hand
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  // MATLAB and serial monitor have to use the same baud
  Serial.begin(115200);

  // detects rising and falling edges of A
  // B is chekced inside the interrupt function
  attachInterrupt(digitalPinToInterrupt(LEFT_A), readLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), readRightEncoder, CHANGE);

  // Establishes start time
  // initializes update timer
  startTime = millis();
  lastUpdate = startTime;
  lastPrint = startTime;

  // Numeric output only
  // Outputs for MATLAB (makes it easier)
  Serial.println("0.000\t0.00000\t0.00000\t0.00000");
}

void loop() {
  unsigned long now = millis();

  // Update localization every 10 milliseconds.
  if (now - lastUpdate >= 10) {
    lastUpdate = now;

    // Pauses interrupts while copying shared counters
    // Prevents interrupt from changing partway through
    noInterrupts();
    long currentLeft = leftCount;
    long currentRight = rightCount;
    interrupts();

    // Finds how many counts each wheel has moved since the previous update
    long deltaLeft = currentLeft - previousLeft;
    long deltaRight = currentRight - previousRight;

    // saves the current counts for next update
    previousLeft = currentLeft;
    previousRight = currentRight;


    // calculations from tutorial
    float metersPerCount = 2.0 * PI * WHEEL_RADIUS / COUNTS_PER_REV;

    float leftDistance = LEFT_SIGN * (float)deltaLeft * metersPerCount;

    float rightDistance = RIGHT_SIGN * (float)deltaRight * metersPerCount;

    float distance = (leftDistance + rightDistance) / 2.0;

    // Use the old heading for both position updates.
    x += distance * cos(heading);
    y += distance * sin(heading);

    // Positive heading is counterclockwise.
    heading += (rightDistance - leftDistance) / WHEEL_SPACING;

    // Print approximately every 100 milliseconds.
    if (now - lastPrint >= 100) {
      lastPrint = now;

      // converts elapsed time to sec.
      Serial.print((now - startTime) / 1000.0, 3);
      Serial.print('\t');
      // sends position in meters and heading in radians
      Serial.print(x, 5);
      Serial.print('\t');
      Serial.print(y, 5);
      Serial.print('\t');
      Serial.println(heading, 5);
    }
  }
}