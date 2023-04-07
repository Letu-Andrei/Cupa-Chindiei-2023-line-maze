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
#define robotStatePin 12
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
const double kP = 100.0 / 10000;
const double kI =   0.0 / 500000000;
const double kD = 300.0 * 10;
// to be tuned
// measuring distance by time until we implement encoders
const int checkIntersectionDelay = 50;
const int  leftTurnDelay = 1000;
const int rightTurnDelay = leftTurnDelay;

// PID
// correction will be positive if the robot needs to turn left
int16_t error;
int16_t lastError;
int64_t errorSum;
int16_t correction;
int64_t timeNow;
int64_t lastTime;
int64_t microsecondsElapsed;
const int32_t maxTimeDifMicros = 200000;

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

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  // initialize motor speeds
  pinMode(standBy, OUTPUT);
  digitalWrite(standBy, HIGH);
   leftMotor.stop();
  rightMotor.stop();

  // configure line sensors
  lineSensors.setTypeAnalog();
  lineSensors.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, lineSensorCount);
  // calibrateLineSensors();
  autoCalibrateLineSensors();

  // initialize robot in line follow state
  robotState = mazeExplorationState;

  // initialize intersections[0] with a non-0 value to prevent index from going below 1 when shortening path
  intersections[0] = intersectionForward;
  intersectionCount = 1;

  // initialize PID timer
  lastTime = micros();

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

  for (uint16_t i = 0; i < 400; i++)
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
  for (uint16_t i = 0; i < 200; i++)
  {
    lineSensors.calibrate();
  }

  setLeftMotorSpeed(-50);
  setRightMotorSpeed(50);
  for (uint16_t i = 0; i < 200; i++)
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
  // displayLineSensorValues();
  digitalWrite(LED_BUILTIN, HIGH);
  turnLeft();
  // setLeftMotorSpeed(100);
  // setRightMotorSpeed(-100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);  
}

void doMazeExploration() {
  readLinePosition();
  displayLineSensorValues();

  if (leftTheLine) {
    // intersectionCount--;
    turnLeft();
    turnLeft();
    return;
  }

  calculatePID();

  // calculate new speeds based on correction (WIP)
   leftSpeed = baseSpeed - correction;
  rightSpeed = baseSpeed + correction;

  // set new motor speeds
  setLeftMotorSpeed(leftSpeed);
  setRightMotorSpeed(rightSpeed);

  if (lineSensorValues[0] + lineSensorValues[lineSensorCount - 1] >= 600) {
    // intersectionCount++;
    canTurnLeft = lineSensorValues[0] > 500;
    delay(checkIntersectionDelay);

    if (lineSensorValues[0] + lineSensorValues[lineSensorCount - 1] >= 1200) {
      // finished the maze
      delay(100000);
    }
    else if (canTurnLeft) {
      intersections[intersectionCount] += intersectionLeft;
      turnLeft();
    }
    else if (lineSensorValues[lineSensorCount / 2 - 1] + lineSensorValues[lineSensorCount / 2] >= 1200) {
      intersections[intersectionCount] += intersectionForward;
      return;
    }
    else {
      intersections[intersectionCount] += intersectionRight;
      turnRight();
    }
  }
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
  leftMotor.stop();
  rightMotor.stop();
  delay(100);
  leftMotor.reset();
  leftMotor.runFor(leftTurnDelay, 1);
  rightMotor.reset();
  rightMotor.runFor(leftTurnDelay, 0);
  delay(100);
}

void turnRight() {
  leftMotor.stop();
  rightMotor.stop();
  delay(100);
  leftMotor.reset();
  leftMotor.runFor(rightTurnDelay, 0);
  rightMotor.reset();
  rightMotor.runFor(rightTurnDelay, 1);
  delay(100);
}
