// ===================== Pin Definitions ===================== //

#define ENA_PIN 5  // Enable pin for Motor A (right motor)
#define IN1_PIN 3  // IN1 for Motor A
#define IN2_PIN 2  // IN2 for Motor A
#define IN3_PIN 7  // IN3 for Motor B (left motor)
#define IN4_PIN 4  // IN4 for Motor B
#define ENB_PIN 6  // Enable pin for Motor B

// Control Buttons
#define BUTTON1_PIN 11  // Button 1: Dynamic Calibration
#define BUTTON2_PIN 10  // Button 2: Unused (for future use)
#define BUTTON3_PIN 9   // Button 3: Left-hand Maze Scan
#define BUTTON4_PIN 8   // Button 4: Solve Maze

// LEDs
#define white_blue_led 12  // Indicator LED
#define RED_LED 13         // Status LED (Built-in on Arduino Nano)

// Sensor Definitions
#define NUM_SENSORS 8
const int sensorPins[NUM_SENSORS] = { A0, A1, A2, A3, A4, A5, A6, A7 };  // A0 to A7

// ===================== Global Variables ===================== //

bool isCurrentlyTurning = false;  // Flag to track if robot is executing a turn
unsigned long lastTurnTime = 0;

bool wasOnLine = true;  // Start as true since robot starts on a line

// Add a startup delay flag
bool startupDelay = true;
unsigned long startTime = 0;


int sensorValue[NUM_SENSORS];
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorThreshold[NUM_SENSORS];  // Dynamic thresholds

bool BUTTON1Pressed = false;       // Track if Button 1 has been pressed
bool BUTTON2Pressed = false;       // Track if Button 2 has been pressed
bool BUTTON3Pressed = false;       // Track if Button 3 has been pressed
bool BUTTON4Pressed = false;       // Track if Button 4 has been pressed
bool lineFollowingActive = false;  // Track if line following is active


// PID Variables
float current_error = 0, previous_error = 0;
float accumulated_error = 0;
int K_p = 60;               // Proportional Gain
int K_i = 0.2;              // Integral Gain
int K_d = 1000;             // Derivative Gain
int base_motor_speed = 65;  // Base motor speed
int PID_value = 0;

const int MAX_PATH_LENGTH = 100;
char path[MAX_PATH_LENGTH];     // Array to store path directions
int pathLength = sizeof(path);  // effectivePathLength excluding null terminator
int pathIndex = 0;
int effectivePathLength = pathLength;  // Track effective effectivePathLength after



// ===================== Function Definitions ================== //


// Function to store a direction in RAM
void storePath(char direction) {
  if (!BUTTON4Pressed) {
    if (pathIndex < MAX_PATH_LENGTH) {
      path[pathIndex] = direction;  // Store in RAM
      pathIndex++;
    } else {
      Serial.println("Path storage is full!");
    }
  }
}

void printPath(char arr[], int effectivePathLength) {
  for (int i = 0; i < effectivePathLength; i++) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println();
}

// Function to replace sequences in path array and return new effectivePathLength
// Function to shorten the path by replacing sequences based on LSRB rules
int shortenPath(char path[], int effectivePathLength) {
    bool optimized = true;

    while (optimized) {
        optimized = false;
        for (int i = 0; i < effectivePathLength - 2; i++) {
            if (replacePattern(path, i)) {
                for (int j = i + 1; j < effectivePathLength - 2; j++) {
                    path[j] = path[j + 2];
                }
                effectivePathLength -= 2;
                optimized = true;
                break;  // Restart after each replacement
            }
        }
    }

    // Ensure the path is null-terminated
    for (int i = effectivePathLength; i < MAX_PATH_LENGTH; i++) {
        path[i] = '\0';
    }

    return effectivePathLength;  // Return new effective effectivePathLength of the path
}

// Function to replace sequences based on LSRB rules
bool replacePattern(char path[], int index) {
  char a = path[index];
  char b = path[index + 1];
  char c = path[index + 2];

  if (a == 'L' && b == 'B' && c == 'R') {
    path[index] = 'B';
    return true;
  } else if (a == 'R' && b == 'B' && c == 'L') {
    path[index] = 'B';
    return true;
  } else if (a == 'S' && b == 'B' && c == 'L') {
    path[index] = 'R';
    return true;
  } else if (a == 'L' && b == 'B' && c == 'L') {
    path[index] = 'S';
    return true;
  } else if (a == 'R' && b == 'B' && c == 'R') {
    path[index] = 'S';
    return true;
  } else if (a == 'S' && b == 'B' && c == 'R') {
    path[index] = 'L';
    return true;
  } else if (a == 'L' && b == 'B' && c == 'S') {
    path[index] = 'R';
    return true;
  } else if (a == 'R' && b == 'B' && c == 'S') {
    path[index] = 'L';
    return true;
  } else if (a == 'S' && b == 'B' && c == 'S') {
    path[index] = 'B';
    return true;
  }

  return false;  // No replacement made
}

// Function to update sensor values
void updateSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValue[i] = analogRead(sensorPins[i]);
  }
}
//Function to check all sensor below threshold
bool onwhite() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    updateSensors();
    if (sensorValue[i] < sensorThreshold[i]) {
      return true;
    } else return false;
  }
}

// Dynamic sensor calibration (from Code 1)
void calibrateSensors() {
  Serial.println("Dynamic calibration started...");
  digitalWrite(white_blue_led, HIGH);

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    // Move in circle for calibration
    moveInCircle();

    for (int i = 0; i < NUM_SENSORS; i++) {
      int sensorValue = analogRead(sensorPins[i]);
      if (sensorValue < sensorMin[i]) sensorMin[i] = sensorValue;
      if (sensorValue > sensorMax[i]) sensorMax[i] = sensorValue;
    }
  }

  stopMotors();

  // Calculate dynamic thresholds
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }

  Serial.println("Dynamic calibration complete.");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": Threshold = ");
    Serial.println(sensorThreshold[i]);
  }

  digitalWrite(white_blue_led, LOW);
  // Serial.println("Calibration started...");
  // sensorThreshold[0] = 450;
  // sensorThreshold[1] = 216;
  // sensorThreshold[2] = 278;
  // sensorThreshold[3] = 203;
  // sensorThreshold[4] = 325;
  // sensorThreshold[5] = 325;
  // sensorThreshold[6] = 361;
  // sensorThreshold[7] = 346;
  // Serial.println("Calibration completed.");
}

// Move robot in a circle for calibration
void moveInCircle() {
  analogWrite(ENA_PIN, 55);
  analogWrite(ENB_PIN, 55);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
}


// Calculate line position
float calculateCenter() {
  float positionSum = 0;
  float weightSum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    positionSum += sensorValue[i] * (i + 1);
    weightSum += sensorValue[i];
  }

  return (weightSum > 0) ? (positionSum / weightSum) : 0;
}
// Function to check if all sensors are detecting white (possible dead end)
bool allSensorsOffLine() {
  int offLineSensors = 0;
  // Check all sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValue[i] < sensorThreshold[i]) {
      offLineSensors++;
    }
  }
  // Return true if most sensors are off the line (allowing for some noise)
  return offLineSensors >= 6;  // Adjust this threshold if needed
}

// Function to check if we were recently on a valid line
bool wasRecentlyOnLine() {
  // Check middle sensors for recent line detection
  return (sensorValue[3] > sensorThreshold[3] || sensorValue[4] > sensorThreshold[4]);
}

// Function to check if we're actually on a line
bool isOnLine() {
  int onLineSensors = 0;
  // Check middle sensors (2,3,4,5) for line detection
  for (int i = 2; i <= 5; i++) {
    if (sensorValue[i] > sensorThreshold[i]) {
      onLineSensors++;
    }
  }
  return onLineSensors > 0;  // Return true if any middle sensor detects line
}

// Modified startRobotForLeftHand function with initialization
void startRobotForLeftHand() {
  // Reset startup flags
  startupDelay = true;
  startTime = 0;
  wasOnLine = true;

  // Small delay before starting
  delay(500);

  while (lineFollowingActive) {
    digitalWrite(white_blue_led, HIGH);

    // Update sensor values
    updateSensors();

    // Check if we're on the line
    bool currentlyOnLine = false;
    for (int i = 2; i <= 5; i++) {
      if (sensorValue[i] > sensorThreshold[i]) {
        currentlyOnLine = true;
        break;
      }
    }

    // Update wasOnLine status
    if (currentlyOnLine) {
      wasOnLine = true;
    }

    // Regular line following and left hand rule
    lineFollow();
    leftHandRule();
  }
}

// Modified startRobotForRightHand function with initialization
void startRobotForRightHand() {
  // Reset startup flags
  startupDelay = true;
  startTime = 0;
  wasOnLine = true;

  // Small delay before starting
  delay(500);

  while (lineFollowingActive) {
    digitalWrite(white_blue_led, HIGH);

    // Update sensor values
    updateSensors();

    // Check if we're on the line
    bool currentlyOnLine = false;
    for (int i = 2; i <= 5; i++) {
      if (sensorValue[i] > sensorThreshold[i]) {
        currentlyOnLine = true;
        break;
      }
    }

    // Update wasOnLine status
    if (currentlyOnLine) {
      wasOnLine = true;
    }

    // Regular line following and right hand rule
    lineFollow();
    RightHandRule();
  }
}

// Modified isDeadEnd function with reduced forward movement
bool isDeadEnd() {
  static const unsigned long MIN_TIME_BETWEEN_TURNS = 90;  // Minimum 1.5 seconds between turns

  // Startup protection
  if (startupDelay) {
    if (startTime == 0) {
      startTime = millis();
    }
    if (millis() - startTime < 125) {  // 2 second startup delay
      return false;
    }
    startupDelay = false;
  }

  // Don't check for dead end if:
  // 1. We're currently turning
  // 2. It's too soon after last turn
  // 3. We're currently on a valid line
  if (isCurrentlyTurning || (millis() - lastTurnTime < MIN_TIME_BETWEEN_TURNS) || isOnLine()) {
    return false;
  }

  // Count how many sensors detect white
  int sensorsOffLine = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValue[i] < sensorThreshold[i]) {
      sensorsOffLine++;
    }
  }

  // Only consider dead end if:
  // 1. Almost all sensors are off the line
  // 2. We were previously on a line (wasOnLine flag is true)
  if (sensorsOffLine >= 7 && wasOnLine) {
    // Move forward very slightly to confirm
    analogWrite(ENA_PIN, 40);  // Reduced speed
    analogWrite(ENB_PIN, 40);
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    delay(280);  // Reduced from 100ms to 50ms
    stopMotors();

    updateSensors();

    // Count sensors again
    sensorsOffLine = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValue[i] < sensorThreshold[i]) {
        sensorsOffLine++;
      }
    }

    if (sensorsOffLine >= 7) {
      // Backup slightly before U-turn
      analogWrite(ENA_PIN, 40);
      analogWrite(ENB_PIN, 40);
      digitalWrite(IN1_PIN, LOW);
      digitalWrite(IN2_PIN, HIGH);
      digitalWrite(IN3_PIN, LOW);
      digitalWrite(IN4_PIN, HIGH);
      delay(70);  // Backup for 100ms
      stopMotors();
      delay(10);
      return true;
    }
  }

  return false;
}
// Modified line following function with deada end detection
void lineFollow() {
  updateSensors();

  // Check if we're currently on the line (using middle sensors)
  bool currentlyOnLine = (sensorValue[3] > sensorThreshold[3] || sensorValue[4] > sensorThreshold[4]);

  // Update wasOnLine status if we're on the line
  if (currentlyOnLine) {
    wasOnLine = true;
  }

  // Check for dead end
  if (!BUTTON4Pressed) {
    if (isDeadEnd()) {
      Serial.println("Dead end detected - Making U-turn");
      makeUTurn();
      wasOnLine = false;        // Reset the line tracking after U-turn
      lastTurnTime = millis();  // Update last turn time
      return;
    }
  }

  // Regular PID line following code
  float currentPosition = calculateCenter();
  float desiredPosition = 4.5;  // Center of 8 sensors

  current_error = desiredPosition - currentPosition;
  PID_value = (K_p * current_error) + (K_d * (current_error - previous_error)) + (K_i * accumulated_error);

  previous_error = current_error;
  accumulated_error += current_error;

  int leftMotorSpeed = base_motor_speed + PID_value;
  int rightMotorSpeed = base_motor_speed - PID_value;

  leftMotorSpeed = constrain(leftMotorSpeed, 0, 65);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 65);

  analogWrite(ENA_PIN, rightMotorSpeed);
  analogWrite(ENB_PIN, leftMotorSpeed);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}
void makeUTurn() {


  storePath('B');  // Store 'B' for U-turn
  isCurrentlyTurning = true;
  Serial.println("Executing left turn...");
  analogWrite(ENA_PIN, 48);
  analogWrite(ENB_PIN, 48);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  bool leftdone = false;

  while (true) {
    updateSensors();
    if (sensorValue[7] > sensorThreshold[7]) {
      leftdone = true;
    }
    if (leftdone && sensorValue[4] > sensorThreshold[4]) {
      stopMotors();
      break;
    }
  }
  isCurrentlyTurning = false;
  lastTurnTime = millis();  // Update last turn time
  wasOnLine = false;        // Reset line tracking after turn
}
// Modify turn functions to update lastTurnTime
void turnLeft() {
  storePath('L');  // Store 'L' for left turn
  isCurrentlyTurning = true;
  Serial.println("Executing left turn...");
  analogWrite(ENA_PIN, 55);
  analogWrite(ENB_PIN, 55);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  bool leftdone = false;

  while (true) {
    updateSensors();
    if (sensorValue[7] > sensorThreshold[7]) {
      leftdone = true;
    }
    if (leftdone && sensorValue[4] > sensorThreshold[4]) {
      stopMotors();
      break;
    }
  }
  isCurrentlyTurning = false;
  lastTurnTime = millis();  // Update last turn time
  wasOnLine = false;        // Reset line tracking after turn
}

void turnRight() {
  storePath('R');  // Store 'R' for right turn
  isCurrentlyTurning = true;
  Serial.println("Executing right turn...");
  analogWrite(ENA_PIN, 55);
  analogWrite(ENB_PIN, 55);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  bool rightdone = false;

  while (true) {
    updateSensors();
    if (sensorValue[0] > sensorThreshold[0]) {
      rightdone = true;
    }
    if (rightdone && sensorValue[3] > sensorThreshold[3]) {
      stopMotors();
      break;
    }
  }
  isCurrentlyTurning = false;
  lastTurnTime = millis();  // Update last turn time
  wasOnLine = false;        // Reset line tracking after turn
}

// Function to move forward
void forward() {
  analogWrite(ENA_PIN, 50);
  analogWrite(ENB_PIN, 50);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}
// Function for instant stop (brake)
void stopMotors() {
  // Apply reverse voltage for Motor A (right motor)
  digitalWrite(IN1_PIN, LOW);                  // Reverse direction
  digitalWrite(IN2_PIN, HIGH);                 // Reverse direction
  analogWrite(ENA_PIN, base_motor_speed - 5);  // Set motor speed for braking

  // Apply reverse voltage for Motor B (left motor)
  digitalWrite(IN3_PIN, LOW);                  // Reverse direction
  digitalWrite(IN4_PIN, HIGH);                 // Reverse direction
  analogWrite(ENB_PIN, base_motor_speed - 5);  // Set motor speed for braking

  // Brief delay for braking action
  delay(40);  // Adjust delay if needed

  // Apply reverse voltage for Motor A (right motor)
  digitalWrite(IN1_PIN, LOW);                  // Reverse direction
  digitalWrite(IN2_PIN, HIGH);                 // Reverse direction
  analogWrite(ENA_PIN, base_motor_speed - 3);  // Set motor speed for braking

  // Apply reverse voltage for Motor B (left motor)
  digitalWrite(IN3_PIN, LOW);                  // Reverse direction
  digitalWrite(IN4_PIN, HIGH);                 // Reverse direction
  analogWrite(ENB_PIN, base_motor_speed - 3);  // Set motor speed for braking

  // Brief delay for braking action
  delay(20);  // Adjust delay if needed
    // Apply reverse voltage for Motor A (right motor)
  digitalWrite(IN1_PIN, LOW);              // Reverse direction
  digitalWrite(IN2_PIN, HIGH);             // Reverse direction
  analogWrite(ENA_PIN, base_motor_speed);  // Set motor speed for braking

  // Apply reverse voltage for Motor B (left motor)
  digitalWrite(IN3_PIN, LOW);              // Reverse direction
  digitalWrite(IN4_PIN, HIGH);             // Reverse direction
  analogWrite(ENB_PIN, base_motor_speed);  // Set motor speed for braking

  // Brief delay for braking action
  delay(10);  // Adjust delay if needed

  // Completely stop the motors after braking
  analogWrite(ENA_PIN, 0);     // Disable Motor A
  analogWrite(ENB_PIN, 0);     // Disable Motor B
  digitalWrite(IN1_PIN, LOW);  // Stop Motor A
  digitalWrite(IN2_PIN, LOW);  // Stop Motor A
  digitalWrite(IN3_PIN, LOW);  // Stop Motor B
  digitalWrite(IN4_PIN, LOW);  // Stop Motor B
}

//###################FOR LEFT HAND MAZE SCAN_#####################
// Function to start robot for left-hand maze solving

// Left hand rule algorithm
void leftHandRule() {
  // ##### Check for Left Turn First #######
  if ((sensorValue[7] > sensorThreshold[7] || sensorValue[6] > sensorThreshold[6]) && !(sensorValue[2] > sensorThreshold[2] || sensorValue[1] > sensorThreshold[1] || sensorValue[0] > sensorThreshold[0])) {
    Serial.println("L");  // Indicate taking left
    // turnLeft();
    linecheckahead_L_detected_for_leftrule();
  }

  // ##### Check for Right Turn if No Left Detected ######
  else if ((sensorValue[0] > sensorThreshold[0] || sensorValue[1] > sensorThreshold[1]) && !(sensorValue[5] > sensorThreshold[5] || sensorValue[7] > sensorThreshold[7] || sensorValue[6] > sensorThreshold[6])) {
    Serial.println("R detected, checking T-intersection and straight path.");

    // Call linecheckahead_R_detected to update detection variables
    linecheckahead_R_detected_for_leftrule();
  }
}
//Function to check other lines ahead
void linecheckahead_R_detected_for_leftrule() {
  bool leftAlsoDetected = false;
  bool goal = false;
  bool straight = false;


  stopMotors();
  delay(35);  // Allow sensors to stabilize
  updateSensors();
  delay(5);

  // Check for a T-intersection by detecting both left and right
  if ((sensorValue[7] > sensorThreshold[7] || sensorValue[6] > sensorThreshold[6]) &&  // Left detected
      (sensorValue[0] > sensorThreshold[0] || sensorValue[1] > sensorThreshold[1])) {  // Right detected
    leftAlsoDetected = true;                                                           // Indicate T-intersection
    Serial.println("T-intersection detected (left and right paths available).");
  } else {
    leftAlsoDetected = false;
  }

  // Move forward slightly to check for goal
  forward();
  delay(330);

  // Check if robot is on white (goal not reached) or black (goal reached)
  bool check = onwhite();
  goal = !check;  // Goal is true if on black


  // Only check for a straight path if no T-intersection (left) is detected
  if (!leftAlsoDetected) {
    if (sensorValue[3] > sensorThreshold[3] || sensorValue[4] > sensorThreshold[4] || sensorValue[5] > sensorThreshold[5] || sensorValue[2] > sensorThreshold[2]) {
      straight = true;
      Serial.println("Straight path detected");
    } else {
      straight = false;
    }
  } else {
    straight = false;  // Reset straight if T-intersection is detected
  }


  // Prioritize T-intersection (left turn) if detected
  if (goal) {
    storePath('G');

    digitalWrite(RED_LED, HIGH);
    digitalWrite(white_blue_led, LOW);
    lineFollowingActive = false;
    stopMotors();
    checkButtons();
  } else if (leftAlsoDetected) {
    Serial.println("T is there. Taking left.");
    turnLeft();
  } else if (straight) {
    Serial.println("Following straight line.");
    storePath('S');  // Store 'S' for straight path if needed
    lineFollow();
  } else {
    Serial.println("Turning right.");
    turnRight();
  }
}

void linecheckahead_L_detected_for_leftrule() {

  bool rightAlsoDetected = false;
  bool goal = false;


  stopMotors();
  delay(35);  // Allow sensors to stabilize
  updateSensors();
  delay(5);

  // Check for a T-intersection by detecting both left and right
  if ((sensorValue[7] > sensorThreshold[7] || sensorValue[6] > sensorThreshold[6]) &&  // Left detected
      (sensorValue[0] > sensorThreshold[0] || sensorValue[1] > sensorThreshold[1])) {  // Right detected
    rightAlsoDetected = true;                                                          // Indicate T-intersection
    Serial.println("T-intersection detected (left and right paths available).");
  } else {
    rightAlsoDetected = false;
  }

  // Move forward slightly to check for goal
  forward();
  delay(330);

  // Check if robot is on white (goal not reached) or black (goal reached)
  bool check = onwhite();
  goal = !check;  // Goal is true if on black


  // // Only check for a straight path if no T-intersection (left) is detected
  // if (!leftAlsoDetected) {
  //   if (sensorValue[3] > sensorThreshold[3] || sensorValue[4] > sensorThreshold[4] || sensorValue[5] > sensorThreshold[5] || sensorValue[2] > sensorThreshold[2]) {
  //     straight = true;
  //     Serial.println("Straight path detected");
  //   } else {
  //     straight = false;
  //   }
  // } else {
  //   straight = false;  // Reset straight if T-intersection is detected
  // }


  // Prioritize T-intersection (left turn) if detected
  if (goal) {
    storePath('G');
    digitalWrite(RED_LED, HIGH);
    digitalWrite(white_blue_led, LOW);
    lineFollowingActive = false;
    stopMotors();
    checkButtons();
  } else if (rightAlsoDetected) {
    Serial.println("T is there. Taking left.");
    turnLeft();
  } else {
    Serial.println("Turning right.");
    turnLeft();
  }
}



//###################FOR RIGHT HAND MAZE SCAN_#####################

// Function to start robot for left-hand maze solving

void RightHandRule() {

  // ##### Check for Left Turn First #######
  if ((sensorValue[7] > sensorThreshold[7] || sensorValue[6] > sensorThreshold[6]) && !(sensorValue[2] > sensorThreshold[2] || sensorValue[1] > sensorThreshold[1] || sensorValue[0] > sensorThreshold[0])) {
    Serial.println("L");  // Indicate taking left
    // turnLeft();
    linecheckahead_L_detected_for_rightrule();
  }

  // ##### Check for Right Turn if No Left Detected ######
  else if ((sensorValue[0] > sensorThreshold[0] || sensorValue[1] > sensorThreshold[1]) && !(sensorValue[5] > sensorThreshold[5] || sensorValue[7] > sensorThreshold[7] || sensorValue[6] > sensorThreshold[6])) {
    Serial.println("R detected, checking T-intersection and straight path.");

    // Call linecheckahead_R_detected to update detection variables
    linecheckahead_R_detected_for_rightrule();
  }
}
void linecheckahead_R_detected_for_rightrule() {
  bool leftAlsoDetected = false;
  bool goal = false;


  stopMotors();
  delay(35);  // Allow sensors to stabilize
  updateSensors();
  delay(5);

  // Check for a T-intersection by detecting both left and right
  if ((sensorValue[7] > sensorThreshold[7] || sensorValue[6] > sensorThreshold[6]) &&  // Left detected
      (sensorValue[0] > sensorThreshold[0] || sensorValue[1] > sensorThreshold[1])) {  // Right detected
    leftAlsoDetected = true;                                                           // Indicate T-intersection
    Serial.println("T-intersection detected (left and right paths available).");
  } else {
    leftAlsoDetected = false;
  }

  // Move forward slightly to check for goal
  forward();
  delay(330);

  // Check if robot is on white (goal not reached) or black (goal reached)
  bool check = onwhite();
  goal = !check;  // Goal is true if on black

  // Prioritize T-intersection (left turn) if detected
  if (goal) {
    storePath('G');
    digitalWrite(RED_LED, HIGH);
    digitalWrite(white_blue_led, LOW);
    lineFollowingActive = false;
    stopMotors();
    checkButtons();
  } else if (leftAlsoDetected) {
    Serial.println("T is there. Taking left.");
    turnRight();
  } else {
    Serial.println("Turning right.");
    turnRight();
  }
}

void linecheckahead_L_detected_for_rightrule() {

  bool rightAlsoDetected = false;
  bool goal = false;
  bool straight = false;


  stopMotors();
  delay(35);  // Allow sensors to stabilize
  updateSensors();
  delay(5);

  // Check for a T-intersection by detecting both left and right
  if ((sensorValue[7] > sensorThreshold[7] || sensorValue[6] > sensorThreshold[6]) &&  // Left detected
      (sensorValue[0] > sensorThreshold[0] || sensorValue[1] > sensorThreshold[1])) {  // Right detected
    rightAlsoDetected = true;                                                          // Indicate T-intersection
    Serial.println("T-intersection detected (left and right paths available).");
  } else {
    rightAlsoDetected = false;
  }

  // Move forward slightly to check for goal
  forward();
  delay(330);

  // Check if robot is on white (goal not reached) or black (goal reached)
  bool check = onwhite();
  goal = !check;  // Goal is true if on black


  // Only check for a straight path if no T-intersection (left) is detected
  if (!rightAlsoDetected) {
    if (sensorValue[3] > sensorThreshold[3] || sensorValue[4] > sensorThreshold[4] || sensorValue[5] > sensorThreshold[5] || sensorValue[2] > sensorThreshold[2]) {
      straight = true;
      Serial.println("Straight path detected");
    } else {
      straight = false;
    }
  } else {
    straight = false;  // Reset straight if T-intersection is detected
  }


  // Prioritize T-intersection (left turn) if detected
  if (goal) {
    storePath('G');
    digitalWrite(RED_LED, HIGH);
    digitalWrite(white_blue_led, LOW);
    lineFollowingActive = false;
    stopMotors();
    checkButtons();
  } else if (rightAlsoDetected) {
    Serial.println("T is there. Taking left.");
    turnRight();
  } else if (straight) {
    storePath('S');  // Store 'S' for straight path if needed
    Serial.println("Following straight line.");
    lineFollow();
  } else {
    Serial.println("Turning right.");
    turnLeft();
  }
}

// Function to check buttons and run appropriate tasks
void checkButtons() {
  digitalWrite(RED_LED, LOW);
  if (digitalRead(BUTTON1_PIN) == LOW && !BUTTON1Pressed) {
    delay(500);
    BUTTON1Pressed = true;
    calibrateSensors();  // Dynamic calibration
  }

  if (digitalRead(BUTTON1_PIN) == HIGH) {
    BUTTON1Pressed = false;
  }

  if (digitalRead(BUTTON2_PIN) == LOW && !BUTTON2Pressed) {
    BUTTON2Pressed = true;
    lineFollowingActive = true;
    Serial.println("Start right-hand maze scan");
    startRobotForRightHand();  // Start right-hand maze scan
  }

  if (digitalRead(BUTTON3_PIN) == HIGH) {
    BUTTON2Pressed = false;
  }

  if (digitalRead(BUTTON3_PIN) == LOW && !BUTTON3Pressed) {
    BUTTON3Pressed = true;
    lineFollowingActive = true;
    Serial.println("Start left-hand maze scan");
    startRobotForLeftHand();  // Start left-hand maze scan
  }

  if (digitalRead(BUTTON3_PIN) == HIGH) {
    BUTTON3Pressed = false;
  }

  if (digitalRead(BUTTON4_PIN) == LOW && !BUTTON4Pressed) {
    delay(500);
    BUTTON4Pressed = true;
    BUTTON2Pressed = false;
    BUTTON3Pressed = false;

    Serial.println("Original Path:");
    printPath(path, pathLength);
    // Perform the path shortening
    // int shortenedLength = shortenPath(path, pathLength);
    effectivePathLength = shortenPath(path, effectivePathLength);  // Update effective effectivePathLength

    Serial.println("Shortest Path:");
    printPath(path, effectivePathLength);

    lineFollowingActive = true;
    Serial.println("Maze solve started");
    startRobotFor_MazeSolve();
  }

  if (digitalRead(BUTTON4_PIN) == HIGH) {
    BUTTON4Pressed = false;
  }
}

// Setup function
void setup() {
  Serial.begin(9650);
  pinMode(white_blue_led, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(BUTTON4_PIN, INPUT_PULLUP);

  // Motor Pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
 
  Serial.println("Press Button 1 for dynamic calibration.");
  Serial.println("Press Button 2 for right-hand maze scan.");
  Serial.println("Press Button 3 for left-hand maze scan.");
  Serial.println("Press Button 4 for solve the maze.");
}

// Main loop
void loop() {
  checkButtons();
}
void startRobotFor_MazeSolve() {

  while (lineFollowingActive) {
    digitalWrite(white_blue_led, HIGH);
    updateSensors();
    lineFollow();
    maze();
  }
}

void maze() {


  // ##### Checking for any turns or intersection #######
  if (sensorValue[7] > sensorThreshold[7] || sensorValue[6] > sensorThreshold[6] || sensorValue[1] > sensorThreshold[1] || sensorValue[0] > sensorThreshold[0]) {
    forward();
    delay(180);
    maze_direction();
  }
}
void maze_direction() {
  stopMotors();
  delay(15);
  if (path[pathIndex] == 'R') {
    turnRight();
    pathIndex++;
  } else if (path[pathIndex] == 'L') {
    turnLeft();
    pathIndex++;
  } else if (path[pathIndex] == 'S') {
    lineFollow();
    pathIndex++;
  } else if (path[pathIndex] == 'G') {
    stopMotors();
    delay(1000000);
  }
}
