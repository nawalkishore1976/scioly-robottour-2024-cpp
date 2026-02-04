#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "RB_ENCONDERMOTOR.h" 
#include "RB_BUZZER.h"
#include "RB_RGBLED.h"
#include <ICM20948_WE.h>

// =============================================================================
// DEBUG CONFIGURATION - 0 runtime cost when disabled
// =============================================================================
#define DEBUG_ENABLED 1  // Set to 0 to disable all debug prints with 0 runtime cost

#if DEBUG_ENABLED
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(format, ...) Serial.printf(format, __VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(format, ...)
#endif

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
#define MICROS_PER_SECOND 1000000UL
#define MICROS_PER_MILLI 1000UL

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
#define RGB_LED_PIN 44
#define BUZZER_PIN 45

// Encoder interrupt pins (from RB_EncoderMotor)
#define M1_ENCODER_A 18
#define M1_ENCODER_B 19
#define M2_ENCODER_A 20
#define M2_ENCODER_B 21

// =============================================================================
// HARDWARE OBJECTS
// =============================================================================
RB_EncoderMotor M1(1);  // Left motor (master)
RB_EncoderMotor M2(2);  // Right motor (slave, follows M1)
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
RB_Buzzer Buzzer(BUZZER_PIN);
RB_RGBLed RGBLED(RGB_LED_PIN, 2);

// =============================================================================
// ENCODER INTERRUPT VARIABLES - SIMPLIFIED
// =============================================================================
volatile long m1_count = 0;
volatile long m2_count = 0;

// =============================================================================
// TRACKING VARIABLES
// =============================================================================
long total_m1_step_count = 0;
long current_m1_step_count = 0;
long previous_m1_count = 0;
long previous_m2_count = 0;
float current_turning_speed = 0;  // degrees per millisecond
float current_real_PPS_M1 = 0;    // Pulses per second M1
float current_real_PPS_M2 = 0;    // Pulses per second M2
uint32_t start_time_micros = 0;   // Program start time
uint32_t end_time_micros = 0;     // Program end time

// =============================================================================
// MELODIES FOR BUZZER
// =============================================================================
int melody[] = {262, 294, 330, 349, 392, 440, 494, 523};
int noteDurations[] = {4, 4, 4, 4, 4, 4, 4, 2};
const uint8_t melodyLength = 8;

int successMelody[] = {523, 587, 659, 698, 784};
int successDurations[] = {4, 4, 4, 4, 2};
const uint8_t successLength = 5;

// =============================================================================
// SIMPLE INTERRUPT SERVICE ROUTINES
// =============================================================================
void m1EncoderISR() {
  m1_count++;
}

void m2EncoderISR() {
  m2_count++;
}

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
    uint32_t now = micros();
    float dt = (now - lastTime) / MICROS_PER_SECOND;
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
  
  void setGains(float p, float i, float d) {
    kP = p; kI = i; kD = d;
  }
};

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
  float turnRate; // degrees per millisecond
  
public:
  IMUHelper(ICM20948_WE* imuPtr) : imu(imuPtr), gyroOffsetZ(0), currentHeading(0), lastUpdate(0), turnRate(0) {}
  
  bool begin() {
    Wire.begin();
    if(!imu->init()) {
      DEBUG_PRINTLN("ICM20948 does not respond");
      return false;
    }
    
    DEBUG_PRINTLN("ICM20948 is connected");
    DEBUG_PRINTLN("Position your ICM20948 flat and don't move it - calibrating...");
    RGBLED.setColor(1, 50, 50, 0); // Yellow during calibration
    RGBLED.show();
    delay(1000);
    
    imu->autoOffsets();
    DEBUG_PRINTLN("IMU calibration done!");
    
    imu->setGyrRange(ICM20948_GYRO_RANGE_250);
    imu->setGyrDLPF(ICM20948_DLPF_6);
    
    RGBLED.setColor(1, 0, 50, 0); // Green when ready
    RGBLED.show();
    
    return true;
  }
  
  void update() {
    uint32_t now = micros();
    float dt_micros = (now - lastUpdate);
    if (dt_micros < 10000) return; // Max 100Hz update (10ms = 10000 micros)
    
    float dt_seconds = dt_micros / MICROS_PER_SECOND;
    
    imu->readSensor();
    xyzFloat gyr = imu->getGyrValues();
    
    // Calculate turn rate in degrees per millisecond
    current_turning_speed = abs(gyr.z) / 1000.0f; // deg/s to deg/ms
    turnRate = current_turning_speed;
    
    // Integrate gyro for heading
    float gyroZ = gyr.z * PI / 180.0f; // Convert to rad/s
    currentHeading += gyroZ * dt_seconds;
    currentHeading = normalizeAngle(currentHeading);
    
    lastUpdate = now;
  }
  
  float getHeading() { return currentHeading; }
  float getTurnRate() { return turnRate; } // degrees per millisecond
  void resetHeading() { currentHeading = 0; }
  
private:
  float normalizeAngle(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
  }
};

// Motor Controller Helper with PID Speed Control
class MotorController {
private:
  RB_EncoderMotor* masterMotor;  // M1
  RB_EncoderMotor* slaveMotor;   // M2
  float leftTargetSpeed;
  float rightTargetSpeed;
  uint32_t lastUpdate;
  uint32_t lastSpeedMatchUpdate;
  
  // PID Controllers
  PIDController m1SpeedPID;     // M1 speed control
  PIDController m2SpeedPID;     // M2 speed control  
  PIDController speedMatchPID;  // M2 following M1 PPS
  
  // Current PWM values
  int16_t m1_current_pwm;
  int16_t m2_current_pwm;
  
public:
  MotorController(RB_EncoderMotor* left, RB_EncoderMotor* right) 
    : masterMotor(left), slaveMotor(right), leftTargetSpeed(0), rightTargetSpeed(0), 
      lastUpdate(0), lastSpeedMatchUpdate(0),
      m1SpeedPID(2.5f, 0.8f, 0.1f, 50.0f),    // M1 speed PID
      m2SpeedPID(2.5f, 0.8f, 0.1f, 50.0f),    // M2 speed PID
      speedMatchPID(1.5f, 0.5f, 0.2f, 30.0f), // Speed matching PID
      m1_current_pwm(0), m2_current_pwm(0) {}
  
  void begin() {
    // Set motion mode to PWM mode for direct control
    masterMotor->SetMotionMode(PWM_MODE);
    slaveMotor->SetMotionMode(PWM_MODE);
    
    // Initialize encoders
    masterMotor->SetPulsePos(0);
    slaveMotor->SetPulsePos(0);
    
    // Set ratio for encoder calculations
    masterMotor->SetRatio(1);
    slaveMotor->SetRatio(1);
    
    // Setup encoder interrupts
    pinMode(M1_ENCODER_A, INPUT_PULLUP);
    pinMode(M1_ENCODER_B, INPUT_PULLUP);
    pinMode(M2_ENCODER_A, INPUT_PULLUP);
    pinMode(M2_ENCODER_B, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), m1EncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A), m2EncoderISR, CHANGE);
    
    // Reset tracking variables
    resetCounters();
  }
  
  void resetCounters() {
    total_m1_step_count = 0;
    current_m1_step_count = 0;
    previous_m1_count = 0;
    previous_m2_count = 0;
    current_real_PPS_M1 = 0;
    current_real_PPS_M2 = 0;
  }
  
  void setVelocity(float leftVel, float rightVel) {
    leftTargetSpeed = leftVel;
    rightTargetSpeed = rightVel;
  }
  
  void updateSpeedControl() {
    uint32_t now = micros();
    if ((now - lastSpeedMatchUpdate) < 50000) return; // Update every 50ms
    
    float dt_seconds = (now - lastSpeedMatchUpdate) / MICROS_PER_SECOND;
    
    // Calculate current PPS for both motors
    long m1_delta = m1_count - previous_m1_count;
    long m2_delta = m2_count - previous_m2_count;
    
    current_real_PPS_M1 = m1_delta / dt_seconds;
    current_real_PPS_M2 = m2_delta / dt_seconds;
    
    // Update step counts
    current_m1_step_count = m1_delta;
    total_m1_step_count += m1_delta;
    
    // Calculate target PPS from target velocities
    float target_PPS_M1 = (leftTargetSpeed / CM_PER_COUNT);   // Convert cm/s to PPS
    float target_PPS_M2 = (rightTargetSpeed / CM_PER_COUNT);  // Convert cm/s to PPS
    
    // PID control for M1 speed
    float m1_speed_error = target_PPS_M1 - current_real_PPS_M1;
    float m1_pwm_adjustment = m1SpeedPID.update(m1_speed_error);
    m1_current_pwm += (int16_t)m1_pwm_adjustment;
    m1_current_pwm = constrain(m1_current_pwm, -99, 99);
    
    // For straight line motion (similar target speeds), M2 follows M1's PPS
    if (abs(leftTargetSpeed - rightTargetSpeed) < 2.0f && abs(leftTargetSpeed) > 1.0f) {
      // M2 follows M1 PPS for straight line
      float pps_error = current_real_PPS_M1 - current_real_PPS_M2;
      float m2_match_adjustment = speedMatchPID.update(pps_error);
      
      // Also apply regular speed control
      float m2_speed_error = target_PPS_M2 - current_real_PPS_M2;
      float m2_speed_adjustment = m2SpeedPID.update(m2_speed_error);
      
      // Combine both adjustments (speed matching has priority)
      m2_current_pwm += (int16_t)(m2_speed_adjustment * 0.3f + m2_match_adjustment * 0.7f);
    } else {
      // Independent speed control for turns
      float m2_speed_error = target_PPS_M2 - current_real_PPS_M2;
      float m2_pwm_adjustment = m2SpeedPID.update(m2_speed_error);
      m2_current_pwm += (int16_t)m2_pwm_adjustment;
    }
    
    m2_current_pwm = constrain(m2_current_pwm, -99, 99);
    
    // Apply PWM to motors
    masterMotor->SetTarPWM(m1_current_pwm);
    slaveMotor->SetTarPWM(m2_current_pwm);
    
    previous_m1_count = m1_count;
    previous_m2_count = m2_count;
    lastSpeedMatchUpdate = now;
  }
  
  // Brake function - applies reverse PWM briefly to stop without moving backward
  void applyBrake(uint16_t brake_time_ms = 50) {
    DEBUG_PRINTLN("Applying brake...");
    
    // Store current PWM values
    int16_t prev_m1_pwm = m1_current_pwm;
    int16_t prev_m2_pwm = m2_current_pwm;
    
    // Calculate brake PWM (reverse direction, limited strength)
    int16_t brake_pwm_m1 = 0;
    int16_t brake_pwm_m2 = 0;
    
    if (prev_m1_pwm > 5) brake_pwm_m1 = -min(prev_m1_pwm/2, 30);  // Reverse brake
    else if (prev_m1_pwm < -5) brake_pwm_m1 = -max(prev_m1_pwm/2, -30);
    
    if (prev_m2_pwm > 5) brake_pwm_m2 = -min(prev_m2_pwm/2, 30);  // Reverse brake  
    else if (prev_m2_pwm < -5) brake_pwm_m2 = -max(prev_m2_pwm/2, -30);
    
    // Apply brake PWM
    masterMotor->SetTarPWM(brake_pwm_m1);
    slaveMotor->SetTarPWM(brake_pwm_m2);
    
    uint32_t brake_start = micros();
    while ((micros() - brake_start) < (brake_time_ms * 1000UL)) {
      masterMotor->Loop();
      slaveMotor->Loop();
      delay(1);
    }
    
    // Stop motors completely
    masterMotor->SetTarPWM(0);
    slaveMotor->SetTarPWM(0);
    m1_current_pwm = 0;
    m2_current_pwm = 0;
    
    // Reset PID controllers to prevent windup
    m1SpeedPID.reset();
    m2SpeedPID.reset();
    speedMatchPID.reset();
    
    RGBLED.setColor(1, 50, 0, 0); // Red when braking/stopped
    RGBLED.show();
    
    DEBUG_PRINTF("Brake applied for %dms\n", brake_time_ms);
  }
  
  void stop() {
    applyBrake(75);  // Use brake function for better stopping
    setVelocity(0, 0);
  }
  
  void update() {
    updateSpeedControl();
    
    // Update motor loops
    masterMotor->Loop();
    slaveMotor->Loop();
    masterMotor->UpdateSpeed();
    slaveMotor->UpdateSpeed();
  }
  
  // Get encoder positions in counts
  long getLeftEncoder() { return m1_count; }
  long getRightEncoder() { return m2_count; }
  
  // Get PPS values
  float getLeftPPS() { return current_real_PPS_M1; }
  float getRightPPS() { return current_real_PPS_M2; }
  
  // Get current PWM values
  int16_t getLeftPWM() { return m1_current_pwm; }
  int16_t getRightPWM() { return m2_current_pwm; }
  
  // Reset encoders
  void resetEncoders() {
    noInterrupts();
    m1_count = 0;
    m2_count = 0;
    interrupts();
    
    resetCounters();
    masterMotor->SetPulsePos(0);
    slaveMotor->SetPulsePos(0);
  }
};

// =============================================================================
// AUDIO FEEDBACK HELPER
// =============================================================================
class AudioFeedback {
public:
  static void playStartup() {
    RGBLED.setColor(1, 0, 0, 50); // Blue for startup
    RGBLED.show();
    
    for (int i = 0; i < melodyLength; i++) {
      int noteDuration = 1000 / noteDurations[i];
      Buzzer.tone(melody[i], noteDuration);
      int pauseBetweenNotes = noteDuration * 0.6;
      delay(pauseBetweenNotes);
      Buzzer.noTone();
    }
  }
  
  static void playSuccess() {
    RGBLED.setColor(1, 0, 50, 0); // Green for success
    RGBLED.show();
    
    for (int i = 0; i < successLength; i++) {
      int noteDuration = 1000 / successDurations[i];
      Buzzer.tone(successMelody[i], noteDuration);
      int pauseBetweenNotes = noteDuration * 0.6;
      delay(pauseBetweenNotes);
      Buzzer.noTone();
    }
  }
  
  static void playError() {
    for (int i = 0; i < 3; i++) {
      RGBLED.setColor(1, 50, 0, 0); // Red for error
      RGBLED.show();
      Buzzer.tone(200, 200);
      delay(200);
      Buzzer.noTone();
      RGBLED.setColor(1, 0, 0, 0); // Off
      RGBLED.show();
      delay(200);
    }
  }
  
  static void playReady() {
    Buzzer.tone(1000, 200);
    delay(200);
    Buzzer.noTone();
  }
};

// =============================================================================
// UNIFIED DATA STRUCTURE
// =============================================================================
struct Position {
  float x, y, theta;
  
  inline float distance(const Position& other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    return sqrt(dx * dx + dy * dy);
  }
  
  inline Position lerp(const Position& other, float t) const {
    return {x + t * (other.x - x), y + t * (other.y - y), theta + t * (other.theta - theta)};
  }
  
  inline float angleTo(const Position& other) const {
    return atan2(other.y - y, other.x - x);
  }
  
  inline bool equals(const Position& other, float tolerance = 1.0) const {
    return (abs(x - other.x) < tolerance) && (abs(y - other.y) < tolerance);
  }
  
  inline void setPose(float px, float py, float heading) { x = px; y = py; theta = heading; }
  inline void setWaypoint(float px, float py, float speed = 0) { x = px; y = py; theta = speed; }
  inline float getSpeed() const { return theta; }
  inline float getHeading() const { return theta; }
  inline bool isStop() const { return theta <= 0.1; }
};

struct PathSegment {
  uint8_t type;
  float data;
  Position points[MAX_PATH_POINTS];
  uint8_t count;
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
IMUHelper imuHelper(&myIMU);
MotorController motorController(&M1, &M2);

Position robotPose = {0, 0, 0};
uint32_t lastOdometryUpdate = 0;

PIDController turnPID(3.5f, 0.2f, 1.0f, 50.0f);  // Turn control PID

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
  uint32_t now = micros();
  float dt_micros = (now - lastOdometryUpdate);
  if (dt_micros < 10000) return; // 10ms minimum
  
  float dt_seconds = dt_micros / MICROS_PER_SECOND;
  
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
  uint8_t steps = (uint8_t)(d * 0.5f);
  
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
  
  uint32_t startTime = micros();
  const float tolerance = degToRad(2.0f);
  const uint32_t timeout = 5 * MICROS_PER_SECOND; // 5 seconds
  
  RGBLED.setColor(1, 50, 25, 0); // Orange for turning
  RGBLED.show();
  
  while ((micros() - startTime) < timeout) {
    updateOdometry();
    motorController.update();
    
    float error = normalizeAngle(targetHeading - robotPose.getHeading());
    if (abs(error) < tolerance) break;
    
    float turnSpeed = constrain(turnPID.update(error), -40, 40);
    motorController.setVelocity(-turnSpeed, turnSpeed);
    
    DEBUG_PRINTF("Turn: error=%.2f°, turn_speed=%.1f, rate=%.3f°/ms, PPS:(%.1f,%.1f), PWM:(%d,%d)\n", 
                radToDeg(error), turnSpeed, current_turning_speed, 
                current_real_PPS_M1, current_real_PPS_M2, 
                motorController.getLeftPWM(), motorController.getRightPWM());
    
    delay(20);
  }
  
  motorController.stop();
}

void followPath(const Position* path, uint8_t pathSize, float lookahead, uint32_t endTime) {
  RGBLED.setColor(1, 0, 25, 50); // Cyan for path following
  RGBLED.show();
  
  while (micros() < endTime) {
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
    
    DEBUG_PRINTF("Path: pos=(%.1f,%.1f), PPS=(%.1f,%.1f), steps=%ld, PWM:(%d,%d)\n",
                robotPose.x, robotPose.y, current_real_PPS_M1, current_real_PPS_M2, 
                current_m1_step_count, motorController.getLeftPWM(), motorController.getRightPWM());
    
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
  uint32_t lastChange = micros();
  
  while (clicks == 0 || (micros() - lastChange) < (2 * MICROS_PER_SECOND)) {
    bool currentState = digitalRead(START_BUTTON_PIN);
    
    if (currentState != lastState && (micros() - lastChange) > (50 * MICROS_PER_MILLI)) {
      if (!currentState) {
        clicks++;
        RGBLED.setColor(1, 25, 25, 25); // White flash
        RGBLED.show();
        delay(100);
        RGBLED.setColor(1, 0, 50, 0); // Green
        RGBLED.show();
      }
      lastState = currentState;
      lastChange = micros();
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
  DEBUG_PRINTLN("Robot Tour V3 - PID Speed Control & Smart Braking");
  
  // Record start time
  start_time_micros = micros();
  
  // Initialize pins
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(20, OUTPUT);
  digitalWrite(20, HIGH);
  
  // Initialize RGB LED
  RGBLED.setColor(1, 50, 0, 0); // Red during initialization
  RGBLED.show();
  
  // Startup audio
  AudioFeedback::playStartup();
  
  // Initialize motor controller
  motorController.begin();
  
  // Initialize IMU
  if (!imuHelper.begin()) {
    DEBUG_PRINTLN("IMU failed");
    AudioFeedback::playError();
    while (1) {
      RGBLED.setColor(1, 50, 0, 0); // Red error
      RGBLED.show();
      delay(500);
      RGBLED.setColor(1, 0, 0, 0); // Off
      RGBLED.show();
      delay(500);
    }
  }
  
  // Set initial position
  robotPose.setPose(FIELD_SQUARE_SIZE * START_QUAD + FIELD_SQUARE_SIZE * 0.5f, -14, 0);
  
  // Generate path
  convertToAbsoluteCoords();
  generatePath();
  
  DEBUG_PRINTF("Generated %d segments\n", segmentCount);
  
  // Ready signal
  AudioFeedback::playReady();
  RGBLED.setColor(1, 0, 50, 0); // Green ready
  RGBLED.show();
  
  waitForButton();
  RGBLED.setColor(1, 0, 0, 50); // Blue running
  RGBLED.show();
  
  uint32_t endTime = micros() + (TARGET_SECONDS * MICROS_PER_SECOND);
  
  // Execute path
  for (uint8_t i = 0; i < segmentCount; i++) {
    if (micros() >= endTime) break;
    
    const PathSegment* segment = &pathSegments[i];
    
    if (segment->type == 1) { // Turn
      DEBUG_PRINTF("Executing turn to %.1f degrees\n", radToDeg(segment->data));
      turnTo(segment->data);
    } else { // Follow path
      DEBUG_PRINTF("Following path segment %d with %d points\n", i, segment->count);
      followPath(segment->points, segment->count, 15.0f, endTime);
    }
  }
  
  motorController.stop();
  
  // Record end time
  end_time_micros = micros();
  
  // Calculate total execution time
  float total_time_seconds = (end_time_micros - start_time_micros) / MICROS_PER_SECOND;
  
  AudioFeedback::playSuccess();
  
  DEBUG_PRINTLN("=== PATH EXECUTION COMPLETED ===");
  DEBUG_PRINTF("Total Time: %.3f seconds\n", total_time_seconds);
  DEBUG_PRINTF("Total M1 Steps: %ld\n", total_m1_step_count);
  DEBUG_PRINTF("Final PPS M1: %.1f, M2: %.1f\n", current_real_PPS_M1, current_real_PPS_M2);
  DEBUG_PRINTF("Final PWM M1: %d, M2: %d\n", motorController.getLeftPWM(), motorController.getRightPWM());
  DEBUG_PRINTLN("===============================");
}

void loop() {
  updateOdometry();
  motorController.update();
  
  // Status blink and debug
  static uint32_t lastBlink = 0;
  if ((micros() - lastBlink) > MICROS_PER_SECOND) { // 1 second
    digitalWrite(LIGHT_PIN, !digitalRead(LIGHT_PIN));
    lastBlink = micros();
    
    DEBUG_PRINTF("Pos:(%.1f,%.1f,%.1f°) M1_steps:%ld PPS:(%.1f,%.1f) Turn:%.3f°/ms PWM:(%d,%d)\n",
                robotPose.x, robotPose.y, radToDeg(robotPose.getHeading()),
                current_m1_step_count, current_real_PPS_M1, current_real_PPS_M2,
                current_turning_speed, motorController.getLeftPWM(), motorController.getRightPWM());
  }
  
  delay(10);
}
