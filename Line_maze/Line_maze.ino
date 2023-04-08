// library for the motor driver
// https://www.arduino.cc/reference/en/libraries/l298n/
#include <L298N.h>

// library for qtr line sensors
// https://www.arduino.cc/reference/en/libraries/qtrsensors/
#include <QTRSensors.h>

// https://www.arduino.cc/reference/en/libraries/quadratureencoder/

// the states the robot can be in
#define mazeExplorationState 0
#define mazeRunState 1
#define testingState 2
#define lineFollowState 3

// motor encoder pins
#define  leftEncoderAPin 2
#define  leftEncoderBPin 4
#define rightEncoderAPin 3
#define rightEncoderBPin 5

// motor pins (for the motor driver)
#define  leftMotorSpeedPin 6
#define  leftMotorDirectionPin1  8
#define  leftMotorDirectionPin2  7
#define rightMotorSpeedPin 11
#define rightMotorDirectionPin1 12
#define rightMotorDirectionPin2 10
#define standBy 9

// pin for changing robot state
// #define robotStatePin 12
// pin for saving maze to board memory (TBD)
#define saveMazePin 13

// crossroad decisions
#define intersectionLeft 1
#define intersectionForward 2
#define intersectionRight 3

// line sensors
QTRSensors lineSensors;
const uint8_t lineSensorCount = 8;
uint16_t lineSensorValues[lineSensorCount];
int16_t linePosition;

// line sensors lower limit (between 0 and 1023)
const uint16_t lineSensorLowerThreshold = 100;

// robot state
uint8_t robotState = testingState;

// PID constants
const double kP = 120.0 / 10000;
const double kI =   5.0 / 500000000;
const double kD = 300.0 * 10;
// to be tuned
// measuring distance by time until we implement encoders
const uint16_t checkIntersectionDelay = 100;
const uint16_t preTurnDelay = 250;
const uint16_t  leftTurnDelay = 500;
const uint16_t rightTurnDelay = leftTurnDelay;
const uint16_t turnAngle = 160;

// PID
// correction will be positive if the robot needs to turn left
int16_t error;
int16_t lastError;
int64_t errorSum;
int16_t correction;
int64_t timeNow;
int64_t lastTime;
int64_t microsecondsElapsed;
const int32_t maxTimeDifMicros = 1000000;

// maze
uint8_t intersections[256];
uint8_t intersectionCount;
bool leftTheLine;
bool canTurnLeft;
bool canTurnRight;

// define motors
L298N leftMotor ( leftMotorSpeedPin,  leftMotorDirectionPin1,  leftMotorDirectionPin2);
L298N rightMotor(rightMotorSpeedPin, rightMotorDirectionPin1, rightMotorDirectionPin2);
int16_t  leftSpeed;
int16_t rightSpeed;
int16_t maxSpeed = 255;
const int16_t baseSpeed = 100;
const int16_t turnSpeed = 100;

// encoders
int32_t  leftEncoder;
int32_t rightEncoder;
int32_t previousLeftEncoder;
int32_t previousRightEncoder;

void calibrateLineSensors();
void autoCalibrateLineSensors();
void displayLineSensorValues();
void doTesting();
void doMazeExploration();
void doMazeRun();
void doLineFollow();
void readLinePosition();
void calculatePID();
void setMotorSpeed(L298N&, int16_t);
void setLeftMotorSpeed(int16_t);
void setRightMotorSpeed(int16_t);
void turnLeft();
void turnRight();
void leftEncoderChange();
void rightEncoderChange();

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  // initialize motor speeds
  pinMode(standBy, OUTPUT);
  digitalWrite(standBy, HIGH);
   leftMotor.stop();
  rightMotor.stop();

  // set encoder pins to input
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  // attach encoder functions
  attachInterrupt(digitalPinToInterrupt(2), leftEncoderChange, RISING);
  attachInterrupt(digitalPinToInterrupt(3), rightEncoderChange, RISING);

  // configure line sensors
  lineSensors.setTypeAnalog();
  lineSensors.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, lineSensorCount);
  // calibrate line sensors
  calibrateLineSensors();
  // auto calibration seems buggy
  // autoCalibrateLineSensors();

  // initialize robot in line follow state
  // robotState = testingState;
  robotState = mazeExplorationState;
  // robotState = lineFollowState;

  // initialize intersections[0] with a non-0 value to prevent index from going below 1 when shortening path
  intersections[0] = intersectionForward;
  intersectionCount = 1;

  // initialize PID timer
  lastTime = micros();

  // Serial.print("test");
}

void loop() {
  // the robot is a multi-state machine, it will execute the code corresponding to its current state
  switch (robotState) {
    case testingState:
      doTesting();
      break;
    case mazeExplorationState:
      doMazeExploration();
      break;
    case mazeRunState:
      doMazeRun();
      break;
    case lineFollowState:
      doLineFollow();
      break;
  }
  // delay(0);
}

void calibrateLineSensors() {  
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 200; i++)
  {
    lineSensors.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  
  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < lineSensorCount; i++)
  {
    Serial.print(lineSensors.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < lineSensorCount; i++)
  {
    Serial.print(lineSensors.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void autoCalibrateLineSensors() {
  delay(5000);
  digitalWrite(LED_BUILTIN, HIGH);
  
  setLeftMotorSpeed(50);
  setRightMotorSpeed(-50);
  for (uint16_t i = 0; i < 100; i++)
  {
    lineSensors.calibrate();
  }

  setLeftMotorSpeed(-50);
  setRightMotorSpeed(50);
  for (uint16_t i = 0; i < 100; i++)
  {
    lineSensors.calibrate();
  }
  leftMotor.stop();
  rightMotor.stop();

  digitalWrite(LED_BUILTIN, LOW);
  
  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < lineSensorCount; i++)
  {
    Serial.print(lineSensors.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < lineSensorCount; i++)
  {
    Serial.print(lineSensors.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void displayLineSensorValues() {
  lineSensors.readCalibrated(lineSensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line position
  for (uint8_t i = 0; i < lineSensorCount; i++)
  {
    Serial.print(lineSensorValues[i]);
    Serial.print('\t');
  }
  
  Serial.println();
  // delay(0);
}

void doTesting() {
  Serial.print(leftEncoder);
  Serial.print('\t');
  Serial.print(rightEncoder);
  Serial.println();
}

void doMazeExploration() {
  readLinePosition();
  displayLineSensorValues();

  if (leftTheLine) {
    // intersectionCount--;
    leftMotor.stop();
    rightMotor.stop();
    delay(250);
    turnLeft();
    delay(250);
    turnLeft();
    return;
  }

   canTurnLeft = lineSensorValues[0]                   > 500;
  canTurnRight = lineSensorValues[lineSensorCount - 1] > 500;

  if (canTurnLeft || canTurnRight) {
    // intersectionCount++;
    // delay(checkIntersectionDelay);
     previousLeftEncoder =  leftEncoder;
    previousRightEncoder = rightEncoder;
     leftMotor.stop();
    rightMotor.stop();
    while (leftEncoder - previousLeftEncoder < 80 || rightEncoder - previousRightEncoder < 80) {
      Serial.print("Going Forward");
      Serial.println();      
      leftMotor.setSpeed(100);
      leftMotor.forward();
      rightMotor.setSpeed(100);
      rightMotor.forward();
    }
    leftMotor.stop();
    rightMotor.stop();

    readLinePosition();

    if ((lineSensorValues[0] + lineSensorValues[lineSensorCount - 1] > 1200) && canTurnLeft && canTurnRight) {
      // finished the maze
      leftMotor.stop();
      rightMotor.stop();
      delay(100000);
    }
    else if (canTurnLeft) {
      intersections[intersectionCount] += intersectionLeft;
      delay(preTurnDelay);
      turnLeft();
      return;
    }
    else if (lineSensorValues[lineSensorCount / 2 - 2] + 
             lineSensorValues[lineSensorCount / 2 - 1] + 
             lineSensorValues[lineSensorCount / 2] + 
             lineSensorValues[lineSensorCount / 2 + 1] > 800) {
      intersections[intersectionCount] += intersectionForward;
      return;
    }
    else {
      intersections[intersectionCount] += intersectionRight;
      delay(preTurnDelay);
      turnRight();
      return;
    }
  }  

  calculatePID();

  // calculate new speeds based on correction (WIP)
   leftSpeed = baseSpeed - correction;
  rightSpeed = baseSpeed + correction;

  // set new motor speeds
  setLeftMotorSpeed(leftSpeed);
  setRightMotorSpeed(rightSpeed);
}

void doMazeRun() {
  // TBD
}

void doLineFollow() {
  readLinePosition();
  displayLineSensorValues();
  calculatePID();

  // calculate new speeds based on correction (WIP)
   leftSpeed = baseSpeed - correction;
  rightSpeed = baseSpeed + correction;

  Serial.print(leftSpeed);
  Serial.print("\t");
  Serial.print(rightSpeed);
  Serial.println();

  // set new motor speeds
  setLeftMotorSpeed(leftSpeed);
  setRightMotorSpeed(rightSpeed);
}

void readLinePosition() {
  static int32_t sum, pondSum;

  lineSensors.readCalibrated(lineSensorValues);
  
  // ignore values below threshold
  for (uint8_t i = 0; i < lineSensorCount; i++) {
    if (lineSensorValues[i] < lineSensorLowerThreshold) {
      lineSensorValues[i] = 0;
    }
  }
  
  sum = pondSum = 0;
  for (uint8_t i = 0; i < lineSensorCount; i++) {
    sum += lineSensorValues[i];
    pondSum += (int32_t)lineSensorValues[i] * 1000 * i;
  }

  if (sum == 0) {
    leftTheLine = true;
    return;
  }
  leftTheLine = false;
  linePosition = pondSum / sum;
  
  // normalize line position (mid value becomes 0)
  // for 8 sensors this gives a range between -3500 and 3500
  // negative values means the line is to the right of the robot
  // positive values means the line is to the left of the robot
  linePosition -= 500 * (lineSensorCount - 1);

  // for debugging
  // Serial.print("pondSum is ");
  // Serial.print(pondSum);
  // Serial.print(" and sum is ");
  // Serial.print(sum);
  // Serial.print(". linePosition should be ");
  // Serial.print(pondSum / sum);
  // Serial.println();
  // Serial.print("Line position is ");
  // Serial.print(linePosition);
  // Serial.println();
}

void calculatePID() {
  timeNow = micros();
  microsecondsElapsed = timeNow - lastTime;
  
  if (microsecondsElapsed == 0) {
    return;
  }
  
  correction = 0;
  error = linePosition;
  
  // proportional
  correction += kP * error;

  // integral
  errorSum += error * microsecondsElapsed;
  errorSum = constrain(errorSum, maxTimeDifMicros * (-3500), maxTimeDifMicros * 3500);
  correction += kI * errorSum;

  // derivative  
  correction += kD * (error - lastError) / microsecondsElapsed;

  lastError = error;
  lastTime = timeNow;
}

void setMotorSpeed(L298N& motor, int16_t newSpeed) {
  newSpeed = constrain(newSpeed, -255, 255);
  
  if (newSpeed == 0) {
    motor.stop();
  }
  else if (newSpeed > 0) {
    motor.setSpeed(newSpeed);
    motor.forward();
  }
  else {
    newSpeed = -newSpeed;
    motor.setSpeed(newSpeed);
    motor.backward();
  }
}

void setLeftMotorSpeed(int16_t newSpeed) {
  setMotorSpeed(leftMotor, newSpeed);
}

void setRightMotorSpeed(int16_t newSpeed) {
  setMotorSpeed(rightMotor, newSpeed);
}

void turnLeft() {
  Serial.print("Turning left");
  Serial.println();
  previousLeftEncoder = leftEncoder;
  previousRightEncoder = rightEncoder;
  while (leftEncoder - previousLeftEncoder > -turnAngle || rightEncoder - previousRightEncoder < turnAngle) {
    Serial.print("Turning left");
    Serial.println();
     leftMotor.setSpeed(100);
     leftMotor.backward();
    rightMotor.setSpeed(100);
    rightMotor.forward();
  }
   leftMotor.stop();
  rightMotor.stop();
  // leftMotor.stop();
  // rightMotor.stop();

  // delay(250);
  
  // leftMotor.reset();
  // leftMotor.setSpeed(turnSpeed);
  // leftMotor.backward();
  
  // rightMotor.reset();
  // rightMotor.setSpeed(turnSpeed);
  // rightMotor.forward();

  // delay(leftTurnDelay);
  
  // leftMotor.stop();
  // rightMotor.stop();
  
  // delay(250);
}

void turnRight() {
  Serial.print("Turning right");
  Serial.println();
  previousLeftEncoder = leftEncoder;
  previousRightEncoder = rightEncoder;
  while (leftEncoder - previousLeftEncoder < turnAngle || rightEncoder - previousRightEncoder > -turnAngle) {
    Serial.print("Turning right");
    Serial.println();
     leftMotor.setSpeed(100);
     leftMotor.forward();
    rightMotor.setSpeed(100);
    rightMotor.backward();
  }
   leftMotor.stop();
  rightMotor.stop();
  // leftMotor.stop();
  // rightMotor.stop();

  // delay(250);
  
  // leftMotor.reset();
  // leftMotor.setSpeed(turnSpeed);
  // leftMotor.forward();
  
  // rightMotor.reset();
  // rightMotor.setSpeed(turnSpeed);
  // rightMotor.backward();

  // delay(leftTurnDelay);
  
  // leftMotor.stop();
  // rightMotor.stop();
  
  // delay(250);
}

void leftEncoderChange() {
  if (digitalRead(4) == false) {
    leftEncoder++;
  }
  else {
    leftEncoder--;
  }
}

void rightEncoderChange() {
  if (digitalRead(5) == false) {
    rightEncoder--;
  }
  else {
    rightEncoder++;
  }
}