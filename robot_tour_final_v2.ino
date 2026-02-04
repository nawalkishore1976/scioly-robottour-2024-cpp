#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "RB_ENCONDERMOTOR.h" 
#include <ICM20948_WE.h>

// =============================================================================
// CONFIGURATION & CONSTANTS
// =============================================================================
#define MAX_PATH_POINTS 200
#define MAX_PATH_SEGMENTS 50
#define DRIVE_TRACK_WIDTH 15.0  // cm between wheels
#define WHEEL_DIAMETER 6.5      // cm
#define ENCODER_CPR 1440        // Counts per revolution
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)
#define CM_PER_COUNT (WHEEL_CIRCUMFERENCE / ENCODER_CPR)

// Field constants for Science Olympiad Robot Tour
#define FIELD_SQUARE_SIZE 50.0  // cm per grid square
#define TARGET_SECONDS 40
#define START_QUAD 0
#define FINISH_OFFSET (16.0 / 50.0)

// Pin definitions
#define START_BUTTON_PIN 9
#define LIGHT_PIN 13
#define BEEPER_PIN 12
#define CS_PIN 61
#define ICM20948_ADDR 0x68

// =============================================================================
// HARDWARE OBJECTS
// =============================================================================
RB_EncoderMotor M1(1);  // Left motor
RB_EncoderMotor M2(2);  // Right motor
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

// =============================================================================
// HELPER CLASSES AND WRAPPERS
// =============================================================================

// IMU Helper Class - Wrapper around ICM20948_WE
class IMUHelper {
private:
  ICM20948_WE* imu;
  float gyroOffsetZ;
  float currentHeading;
  uint32_t lastUpdate;
  
public:
  IMUHelper(ICM20948_WE* imuPtr) : imu(imuPtr), gyroOffsetZ(0), currentHeading(0), lastUpdate(0) {}
  
  bool begin() {
    Wire.begin();
    if(!imu->init()) {
      Serial.println("ICM20948 does not respond");
      return false;
    }
    
    Serial.println("ICM20948 is connected");
    Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
    delay(1000);
    imu->autoOffsets();
    Serial.println("IMU calibration done!");
    
    imu->setGyrRange(ICM20948_GYRO_RANGE_250);
    imu->setGyrDLPF(ICM20948_DLPF_6);
    
    return true;
  }
  
  void update() {
    uint32_t now = millis();
    float dt = (now - lastUpdate) * 0.001f;
    if (dt < 0.01f) return; // Max 100Hz update
    
    imu->readSensor();
    xyzFloat gyr = imu->getGyrValues();
    
    // Integrate gyro for heading
    float gyroZ = gyr.z * PI / 180.0f; // Convert to rad/s
    currentHeading += gyroZ * dt;
    currentHeading = normalizeAngle(currentHeading);
    
    lastUpdate = now;
  }
  
  float getHeading() {
    return currentHeading;
  }
  
  void resetHeading() {
    currentHeading = 0;
  }
  
private:
  float normalizeAngle(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
  }
};

// Motor Controller Helper - Wrapper around RB_EncoderMotor
class MotorController {
private:
  RB_EncoderMotor* leftMotor;
  RB_EncoderMotor* rightMotor;
  float leftTargetSpeed;
  float rightTargetSpeed;
  uint32_t lastUpdate;
  
public:
  MotorController(RB_EncoderMotor* left, RB_EncoderMotor* right) 
    : leftMotor(left), rightMotor(right), leftTargetSpeed(0), rightTargetSpeed(0), lastUpdate(0) {}
  
  void begin() {
    // Set motion mode to PWM mode for direct control
    leftMotor->SetMotionMode(PWM_MODE);
    rightMotor->SetMotionMode(PWM_MODE);
    
    // Initialize encoders
    leftMotor->SetPulsePos(0);
    rightMotor->SetPulsePos(0);
    
    // Set ratio for encoder calculations (adjust based on your setup)
    leftMotor->SetRatio(1);
    rightMotor->SetRatio(1);
  }
  
  void setVelocity(float leftVel, float rightVel) {
    leftTargetSpeed = leftVel;
    rightTargetSpeed = rightVel;
    
    // Convert cm/s to PWM values (approximate conversion)
    // Adjust these factors based on your robot's characteristics
    float leftPWM = constrain(leftVel * 2.0f, -99, 99);
    float rightPWM = constrain(rightVel * 2.0f, -99, 99);
    
    leftMotor->SetTarPWM((int16_t)leftPWM);
    rightMotor->SetTarPWM((int16_t)rightPWM);
  }
  
  void stop() {
    setVelocity(0, 0);
    delay(100);
  }
  
  void update() {
    leftMotor->Loop();
    rightMotor->Loop();
    leftMotor->UpdateSpeed();
    rightMotor->UpdateSpeed();
  }
  
  // Get encoder positions in counts
  long getLeftEncoder() { return leftMotor->GetPulsePos(); }
  long getRightEncoder() { return rightMotor->GetPulsePos(); }
  
  // Get current speeds
  double getLeftSpeed() { return leftMotor->GetCurrentSpeed(); }
  double getRightSpeed() { return rightMotor->GetCurrentSpeed(); }
  
  // Reset encoders
  void resetEncoders() {
    leftMotor->SetPulsePos(0);
    rightMotor->SetPulsePos(0);
  }
};

// =============================================================================
// UNIFIED DATA STRUCTURE
// =============================================================================
struct Position {
  float x, y, theta;  // x,y coords + theta (heading/speed/flags)
  
  inline float distance(const Position& other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    return sqrt(dx * dx + dy * dy);
  }
  
  inline Position lerp(const Position& other, float t) const {
    return {
      x + t * (other.x - x), 
      y + t * (other.y - y), 
      theta + t * (other.theta - theta)
    };
  }
  
  inline float angleTo(const Position& other) const {
    return atan2(other.y - y, other.x - x);
  }
  
  inline bool equals(const Position& other, float tolerance = 1.0) const {
    return (abs(x - other.x) < tolerance) && (abs(y - other.y) < tolerance);
  }
  
  inline void setPose(float px, float py, float heading) {
    x = px; y = py; theta = heading;
  }
  
  inline void setWaypoint(float px, float py, float speed = 0) {
    x = px; y = py; theta = speed;
  }
  
  inline float getSpeed() const { return theta; }
  inline float getHeading() const { return theta; }
  inline bool isStop() const { return theta <= 0.1; }
};

struct PathSegment {
  uint8_t type;  // 0 = path, 1 = turn
  float data;    // turn angle for turns
  Position points[MAX_PATH_POINTS];
  uint8_t count;
};

// =============================================================================
// PID CONTROLLER
// =============================================================================
class PIDController {
private:
  float kP, kI, kD, integral, prevError, integralMax;
  uint32_t lastTime;
  
public:
  PIDController(float p, float i, float d, float intMax = 100.0) 
    : kP(p), kI(i), kD(d), integralMax(intMax), integral(0), prevError(0), lastTime(0) {}
  
  float update(float error) {
    uint32_t now = millis();
    float dt = (now - lastTime) * 0.001f;
    if (lastTime == 0) dt = 0.02f;
    lastTime = now;
    
    integral += error * dt;
    integral = constrain(integral, -integralMax, integralMax);
    
    float derivative = (dt > 0) ? (error - prevError) / dt : 0;
    float output = kP * error + kI * integral + kD * derivative;
    
    prevError = error;
    return output;
  }
  
  inline void reset() {
    integral = prevError = 0;
    lastTime = 0;
  }
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
IMUHelper imuHelper(&myIMU);
MotorController motorController(&M1, &M2);

Position robotPose = {0, 0, 0};
uint32_t lastOdometryUpdate = 0;

PIDController turnPID(3.0f, 0.1f, 0.8f, 50.0f);

// Path data
Position pathPoints[] = {
  {0, 0, 30}, {1, 0, 30}, {0, 0, 30}, {0, 1, 30},
  {1, 1, 30}, {0, 1, 30}, {0, 2, 30}, {1, 2, 30},
  {1, 3, 30}, {3, 3, 30}, {3, 2, 30}, {2 + FINISH_OFFSET, 2, 0}
};
const uint8_t pathPointCount = sizeof(pathPoints) / sizeof(pathPoints[0]);

PathSegment pathSegments[MAX_PATH_SEGMENTS];
uint8_t segmentCount = 0;

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================
inline float normalizeAngle(float angle) {
  while (angle > PI) angle -= TWO_PI;
  while (angle < -PI) angle += TWO_PI;
  return angle;
}

inline float radToDeg(float rad) { return rad * 57.2957795f; }
inline float degToRad(float deg) { return deg * 0.0174532925f; }

// =============================================================================
// ODOMETRY
// =============================================================================
void updateOdometry() {
  uint32_t now = millis();
  float dt = (now - lastOdometryUpdate) * 0.001f;
  if (dt < 0.01f) return;
  
  // Update IMU
  imuHelper.update();
  
  // Update position from encoders
  static long lastLeftEncoder = 0, lastRightEncoder = 0;
  long leftEncoder = motorController.getLeftEncoder();
  long rightEncoder = motorController.getRightEncoder();
  
  long leftDelta = leftEncoder - lastLeftEncoder;
  long rightDelta = rightEncoder - lastRightEncoder;
  
  float leftDist = leftDelta * CM_PER_COUNT;
  float rightDist = rightDelta * CM_PER_COUNT;
  float avgDist = (leftDist + rightDist) * 0.5f;
  
  float heading = imuHelper.getHeading();
  robotPose.x += avgDist * cos(heading);
  robotPose.y += avgDist * sin(heading);
  robotPose.theta = heading;
  
  lastLeftEncoder = leftEncoder;
  lastRightEncoder = rightEncoder;
  lastOdometryUpdate = now;
}

// =============================================================================
// PATH GENERATION
// =============================================================================
void convertToAbsoluteCoords() {
  for (uint8_t i = 0; i < pathPointCount; i++) {
    pathPoints[i].x = pathPoints[i].x * FIELD_SQUARE_SIZE + FIELD_SQUARE_SIZE * 0.5f;
    pathPoints[i].y = pathPoints[i].y * FIELD_SQUARE_SIZE + FIELD_SQUARE_SIZE * 0.5f;
  }
}

void interpolatePath(Position* path, uint8_t* pathSize, const Position& start, const Position& end, float speed) {
  float d = start.distance(end);
  uint8_t steps = (uint8_t)(d * 0.5f); // One point every 2cm
  
  for (uint8_t n = 1; n <= steps && *pathSize < MAX_PATH_POINTS - 1; n++) {
    float t = (float)n / (steps + 1);
    path[*pathSize].setWaypoint(
      start.x + t * (end.x - start.x),
      start.y + t * (end.y - start.y),
      speed
    );
    (*pathSize)++;
  }
}

void generatePath() {
  segmentCount = 0;
  
  for (uint8_t i = 0; i < pathPointCount - 1 && segmentCount < MAX_PATH_SEGMENTS; i++) {
    const Position& start = pathPoints[i];
    const Position& end = pathPoints[i + 1];
    
    PathSegment* segment = &pathSegments[segmentCount];
    segment->type = 0;
    segment->count = 1;
    segment->points[0] = start;
    
    interpolatePath(segment->points, &segment->count, start, end, start.getSpeed());
    
    if (segment->count < MAX_PATH_POINTS) {
      segment->points[segment->count] = end;
      segment->count++;
    }
    
    if (i < pathPointCount - 2 && pathPoints[i + 2].equals(start)) {
      segmentCount++;
      
      if (segmentCount < MAX_PATH_SEGMENTS) {
        PathSegment* turnSegment = &pathSegments[segmentCount];
        turnSegment->type = 1;
        turnSegment->data = normalizeAngle(start.angleTo(end) + PI);
        segmentCount++;
      }
    } else {
      segmentCount++;
    }
  }
}

// =============================================================================
// PURE PURSUIT
// =============================================================================
uint8_t findClosest(const Position* path, uint8_t pathSize) {
  uint8_t closest = 0;
  float minDist = robotPose.distance(path[0]);
  
  for (uint8_t i = 1; i < pathSize; i++) {
    float dist = robotPose.distance(path[i]);
    if (dist < minDist) {
      minDist = dist;
      closest = i;
    }
  }
  return closest;
}

float circleIntersect(const Position& p1, const Position& p2, float lookaheadDist) {
  float dx = p2.x - p1.x;
  float dy = p2.y - p1.y;
  float fx = p1.x - robotPose.x;
  float fy = p1.y - robotPose.y;
  
  float a = dx * dx + dy * dy;
  if (a < 0.0001f) return -1;
  
  float b = 2 * (fx * dx + fy * dy);
  float c = (fx * fx + fy * fy) - lookaheadDist * lookaheadDist;
  float discriminant = b * b - 4 * a * c;
  
  if (discriminant >= 0) {
    discriminant = sqrt(discriminant);
    float t2 = (-b + discriminant) / (2 * a);
    float t1 = (-b - discriminant) / (2 * a);
    
    if (t2 >= 0 && t2 <= 1) return t2;
    if (t1 >= 0 && t1 <= 1) return t1;
  }
  
  return -1;
}

Position getLookaheadPoint(const Position* path, uint8_t pathSize, uint8_t closest, float lookaheadDist) {
  for (uint8_t i = closest; i < pathSize - 1; i++) {
    float t = circleIntersect(path[i], path[i + 1], lookaheadDist);
    if (t != -1) {
      return path[i].lerp(path[i + 1], t);
    }
  }
  return path[pathSize - 1];
}

inline float calculateCurvature(const Position& lookahead) {
  float dx = lookahead.x - robotPose.x;
  float dy = lookahead.y - robotPose.y;
  float lookaheadDist = sqrt(dx * dx + dy * dy);
  
  if (lookaheadDist < 0.1f) return 0;
  
  float crossTrackError = sin(atan2(dy, dx) - robotPose.getHeading()) * lookaheadDist;
  return (2 * crossTrackError) / (lookaheadDist * lookaheadDist);
}

// =============================================================================
// MOVEMENT FUNCTIONS
// =============================================================================
void turnTo(float targetHeading) {
  targetHeading = normalizeAngle(targetHeading);
  turnPID.reset();
  
  uint32_t startTime = millis();
  const float tolerance = degToRad(2.0f);
  
  while (millis() - startTime < 5000) {
    updateOdometry();
    motorController.update();
    
    float error = normalizeAngle(targetHeading - robotPose.getHeading());
    if (abs(error) < tolerance) break;
    
    float turnSpeed = constrain(turnPID.update(error), -40, 40);
    motorController.setVelocity(-turnSpeed, turnSpeed);
    delay(20);
  }
  
  motorController.stop();
}

void followPath(const Position* path, uint8_t pathSize, float lookahead, uint32_t endTime) {
  while (millis() < endTime) {
    updateOdometry();
    motorController.update();
    
    uint8_t closest = findClosest(path, pathSize);
    
    if (path[closest].isStop()) break;
    
    Position lookaheadPoint = getLookaheadPoint(path, pathSize, closest, lookahead);
    float curvature = calculateCurvature(lookaheadPoint);
    
    float targetSpeed = path[closest].getSpeed();
    float speedMultiplier = 1.0f / (1.0f + abs(curvature) * 20.0f);
    targetSpeed *= speedMultiplier;
    targetSpeed = max(targetSpeed, 5.0f);
    
    float leftSpeed = targetSpeed * (2 + curvature * DRIVE_TRACK_WIDTH) * 0.5f;
    float rightSpeed = targetSpeed * (2 - curvature * DRIVE_TRACK_WIDTH) * 0.5f;
    
    motorController.setVelocity(leftSpeed, rightSpeed);
    delay(20);
  }
  
  motorController.stop();
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================
uint8_t waitForButton() {
  uint8_t clicks = 0;
  bool lastState = digitalRead(START_BUTTON_PIN);
  uint32_t lastChange = millis();
  
  while (clicks == 0 || millis() - lastChange < 2000) {
    bool currentState = digitalRead(START_BUTTON_PIN);
    
    if (currentState != lastState && millis() - lastChange > 50) {
      if (!currentState) {
        clicks++;
        digitalWrite(LIGHT_PIN, !digitalRead(LIGHT_PIN));
      }
      lastState = currentState;
      lastChange = millis();
    }
    delay(10);
  }
  
  return clicks;
}

// =============================================================================
// MAIN FUNCTIONS
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("Robot Tour V2 - Using RB_EncoderMotor & ICM20948"));
  
  // Initialize pins
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(BEEPER_PIN, OUTPUT);
  pinMode(20, OUTPUT);
  digitalWrite(20, HIGH);
  
  // Startup beep
  digitalWrite(BEEPER_PIN, HIGH);
  delay(100);
  digitalWrite(BEEPER_PIN, LOW);
  
  // Initialize motor controller
  motorController.begin();
  
  // Initialize IMU
  if (!imuHelper.begin()) {
    Serial.println(F("IMU failed"));
    while (1) {
      digitalWrite(LIGHT_PIN, HIGH);
      delay(100);
      digitalWrite(LIGHT_PIN, LOW);
      delay(100);
    }
  }
  
  // Set initial position
  robotPose.setPose(FIELD_SQUARE_SIZE * START_QUAD + FIELD_SQUARE_SIZE * 0.5f, -14, 0);
  
  // Generate path
  convertToAbsoluteCoords();
  generatePath();
  
  Serial.print(F("Generated "));
  Serial.print(segmentCount);
  Serial.println(F(" segments"));
  
  // Ready beep
  digitalWrite(BEEPER_PIN, HIGH);
  delay(200);
  digitalWrite(BEEPER_PIN, LOW);
  
  waitForButton();
  digitalWrite(LIGHT_PIN, LOW);
  
  uint32_t endTime = millis() + TARGET_SECONDS * 1000UL;
  
  // Execute path
  for (uint8_t i = 0; i < segmentCount; i++) {
    if (millis() >= endTime) break;
    
    const PathSegment* segment = &pathSegments[i];
    
    if (segment->type == 1) { // Turn
      turnTo(segment->data);
    } else { // Follow path
      followPath(segment->points, segment->count, 15.0f, endTime);
    }
  }
  
  motorController.stop();
  
  // Success beeps
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(BEEPER_PIN, HIGH);
    delay(200);
    digitalWrite(BEEPER_PIN, LOW);
    delay(200);
  }
}

void loop() {
  updateOdometry();
  motorController.update();
  
  // Status blink and debug
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    digitalWrite(LIGHT_PIN, !digitalRead(LIGHT_PIN));
    lastBlink = millis();
    
    Serial.print(F("Pos:("));
    Serial.print(robotPose.x, 1);
    Serial.print(F(","));
    Serial.print(robotPose.y, 1);
    Serial.print(F(","));
    Serial.print(radToDeg(robotPose.getHeading()), 1);
    Serial.print(F("°) Encoders:("));
    Serial.print(motorController.getLeftEncoder());
    Serial.print(F(","));
    Serial.print(motorController.getRightEncoder());
    Serial.println(F(")"));
  }
  
  delay(10);
}