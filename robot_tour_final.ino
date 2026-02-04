```
#include <SPI.h>
#include <math.h>
#include "RB_ENCONDERMOTOR.h" 

RB_EncoderMotor  M1(1);
RB_EncoderMotor  M2(2);

void setupMotor() {
  // put your setup code here, to run once:
    M1.SetMotionMode(0);
    M2.SetMotionMode(0);
    pinMode(20,OUTPUT);
    digitalWrite(20,HIGH); 
}  

void loopMotor() {
  // put your main code here, to run repeatedly:
   M1.SetMotorPwm(30);
   M2.SetMotorPwm(33);
   delay(5000);

   
   M1.SetMotorPwm(-35);
   M2.SetMotorPwm(-35);
   delay(5000); 
 
  
}


h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68

bool spi=true;

//ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
ICM20948_WE myIMU = ICM20948_WE(&SPI, CS_PIN, spi); 


void setup() {
  //delay(2000); // maybe needed for some MCUs, in particular for startup after power off
  Wire.begin();
  Serial.begin(115200);
  while(!Serial) {}
  
  if(!myIMU.init()){
    Serial.println("ICM20948 does not respond");
  }
  else{
    Serial.println("ICM20948 is connected");
  }

  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();
  Serial.println("Done!"); 
  
  /*  The gyroscope data is not zero, even if you don't move the ICM20948. 
   *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
   *  using the +/- 250 degrees/s range. 
   *  Use either autoOffsets or setGyrOffsets, not both.
   *  You can query the offsets with the function:
   *  xyzFloat getGyrOffsets()
  */
  //myIMU.setGyrOffsets(-115.0, 130.0, 105.0);
  
  /* enables or disables the gyroscope sensor, default: enabled */
  // myIMU.enableGyr(false);

  /*  ICM20948_GYRO_RANGE_250       250 degrees per second (default)
   *  ICM20948_GYRO_RANGE_500       500 degrees per second
   *  ICM20948_GYRO_RANGE_1000     1000 degrees per second
   *  ICM20948_GYRO_RANGE_2000     2000 degrees per second
   */
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  
  /*  Choose a level for the Digital Low Pass Filter or switch it off. 
   *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
   *  
   *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
   *    0              196.6               1125/(1+GSRD) 
   *    1              151.8               1125/(1+GSRD)
   *    2              119.5               1125/(1+GSRD)
   *    3               51.2               1125/(1+GSRD)
   *    4               23.9               1125/(1+GSRD)
   *    5               11.6               1125/(1+GSRD)
   *    6                5.7               1125/(1+GSRD) 
   *    7              361.4               1125/(1+GSRD)
   *    OFF          12106.0               9000
   *    
   *    GSRD = Gyroscope Sample Rate Divider (0...255)
   *    You achieve lowest noise using level 6  
   */
  myIMU.setGyrDLPF(ICM20948_DLPF_6);  
  
  /*  Gyroscope sample rate divider divides the output rate of the gyroscope.
   *  Sample rate = Basic sample rate / (1 + divider) 
   *  It can only be applied if the corresponding DLPF is not OFF!
   *  Divider is a number 0...255
   *  If sample rates are set for the accelerometer and the gyroscope, the gyroscope
   *  sample rate has priority.
   */
  //myIMU.setGyrSampleRateDivider(10);
}

void loopGryro() {
  xyzFloat gyrRaw; 
  xyzFloat gyr;
  myIMU.readSensor();
  myIMU.getCorrectedGyrRawValues(&gyrRaw);
  myIMU.getGyrValues(&gyr);
    
  Serial.println("Raw gyroscope values (x,y,z):");
  Serial.print(gyrRaw.x);
  Serial.print("   ");
  Serial.print(gyrRaw.y);
  Serial.print("   ");
  Serial.println(gyrRaw.z);

  Serial.println("Gyroscope values (x,y,z):");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);
  Serial.println();

  delay(500);
}


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
#define LEFT_MOTOR_PWM 3
#define LEFT_MOTOR_DIR1 4
#define LEFT_MOTOR_DIR2 5
#define RIGHT_MOTOR_PWM 6
#define RIGHT_MOTOR_DIR1 7
#define RIGHT_MOTOR_DIR2 8
#define LEFT_ENCODER_A 18
#define LEFT_ENCODER_B 19
#define RIGHT_ENCODER_A 20
#define RIGHT_ENCODER_B 21
#define START_BUTTON_PIN 9
#define LIGHT_PIN 13
#define BEEPER_PIN 12
#define IMU_CS_PIN 61

// ICM-20498 Register addresses
#define ICM20498_WHO_AM_I 0x75
#define ICM20498_PWR_MGMT_1 0x6B
#define ICM20498_GYRO_XOUT_H 0x43
#define ICM20498_ACCEL_XOUT_H 0x3B

// =============================================================================
// UNIFIED DATA STRUCTURE - MEMORY EFFICIENT
// =============================================================================
struct Position {
  float x, y, theta;  // Universal: x,y coords + theta (heading/speed/flags)
  
  // Distance between two positions (ignores theta)
  inline float distance(const Position& other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    return sqrt(dx * dx + dy * dy);
  }
  
  // Linear interpolation between positions
  inline Position lerp(const Position& other, float t) const {
    return {
      x + t * (other.x - x), 
      y + t * (other.y - y), 
      theta + t * (other.theta - theta)
    };
  }
  
  // Angle from this position to another
  inline float angleTo(const Position& other) const {
    return atan2(other.y - y, other.x - x);
  }
  
  // Check if positions are approximately equal
  inline bool equals(const Position& other, float tolerance = 1.0) const {
    return (abs(x - other.x) < tolerance) && (abs(y - other.y) < tolerance);
  }
  
  // Set as robot pose (x, y, heading)
  inline void setPose(float px, float py, float heading) {
    x = px; y = py; theta = heading;
  }
  
  // Set as waypoint (x, y, speed)
  inline void setWaypoint(float px, float py, float speed = 0) {
    x = px; y = py; theta = speed;
  }
  
  // Get speed (when used as waypoint)
  inline float getSpeed() const { return theta; }
  
  // Get heading (when used as pose)
  inline float getHeading() const { return theta; }
  
  // Check if this is a stop waypoint
  inline bool isStop() const { return theta <= 0.1; }
};

struct PathSegment {
  uint8_t type;  // 0 = path, 1 = turn
  float data;    // turn angle for turns, unused for paths
  Position points[MAX_PATH_POINTS];
  uint8_t count;
};

// =============================================================================
// PID CONTROLLER - OPTIMIZED
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
    float dt = (now - lastTime) * 0.001f;  // Convert to seconds
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
// ICM-20498 IMU CLASS - OPTIMIZED
// =============================================================================
class ICM20498 {
private:
  uint8_t csPin;
  float gyroScale, gyroOffsetZ;
  
  inline uint8_t readRegister(uint8_t reg) {
    digitalWrite(csPin, LOW);
    SPI.transfer(reg | 0x80);
    uint8_t data = SPI.transfer(0x00);
    digitalWrite(csPin, HIGH);
    return data;
  }
  
  inline void writeRegister(uint8_t reg, uint8_t data) {
    digitalWrite(csPin, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(data);
    digitalWrite(csPin, HIGH);
  }
  
  inline int16_t read16bit(uint8_t reg) {
    digitalWrite(csPin, LOW);
    SPI.transfer(reg | 0x80);
    int16_t data = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);
    digitalWrite(csPin, HIGH);
    return data;
  }
  
public:
  ICM20498(uint8_t cs) : csPin(cs), gyroScale(131.0f), gyroOffsetZ(0) {}
  
  bool begin() {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    
    delay(100);
    
    if (readRegister(ICM20498_WHO_AM_I) != 0xEA) {
      Serial.println(F("ICM20498 not found"));
      return false;
    }
    
    writeRegister(ICM20498_PWR_MGMT_1, 0x80); // Reset
    delay(100);
    writeRegister(ICM20498_PWR_MGMT_1, 0x01); // Wake up
    delay(100);
    
    calibrate();
    return true;
  }
  
  void calibrate() {
    Serial.println(F("Calibrating gyroscope..."));
    float sum = 0;
    for (int i = 0; i < 1000; i++) {
      sum += read16bit(ICM20498_GYRO_XOUT_H + 4) / gyroScale;
      delay(2);
    }
    gyroOffsetZ = sum / 1000.0f;
  }
  
  inline float getGyroZ() {
    return (read16bit(ICM20498_GYRO_XOUT_H + 4) / gyroScale) - gyroOffsetZ;
  }
};

// =============================================================================
// GLOBAL VARIABLES - MEMORY OPTIMIZED
// =============================================================================
// Hardware objects
ICM20498 imu(IMU_CS_PIN);

// Encoder variables (volatile for ISR)
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile uint8_t leftLastA = 0;
volatile uint8_t rightLastA = 0;

// Robot state - single unified position
Position robotPose = {0, 0, 0};  // x, y, heading
float gyroHeading = 0;
uint32_t lastOdometryUpdate = 0;

// PID Controllers
PIDController turnPID(3.0f, 0.1f, 0.8f, 50.0f);
PIDController leftMotorPID(2.5f, 0.5f, 0.1f, 100.0f);
PIDController rightMotorPID(2.5f, 0.5f, 0.1f, 100.0f);

// Path data - static allocation for memory efficiency
Position pathPoints[] = {
  {0, 0, 30}, {1, 0, 30}, {0, 0, 30}, {0, 1, 30},
  {1, 1, 30}, {0, 1, 30}, {0, 2, 30}, {1, 2, 30},
  {1, 3, 30}, {3, 3, 30}, {3, 2, 30}, {2 + FINISH_OFFSET, 2, 0}  // Last point speed = 0 (stop)
};
const uint8_t pathPointCount = sizeof(pathPoints) / sizeof(pathPoints[0]);

PathSegment pathSegments[MAX_PATH_SEGMENTS];
uint8_t segmentCount = 0;

// Motor control variables
float leftTargetVel = 0, rightTargetVel = 0;
float leftCurrentVel = 0, rightCurrentVel = 0;
uint32_t lastVelUpdate = 0;

// =============================================================================
// INTERRUPT SERVICE ROUTINES - OPTIMIZED
// =============================================================================
void leftEncoderISR() {
  uint8_t currentA = digitalRead(LEFT_ENCODER_A);
  uint8_t currentB = digitalRead(LEFT_ENCODER_B);
  
  if (currentA != leftLastA) {
    leftEncoderCount += (currentA == currentB) ? 1 : -1;
  }
  leftLastA = currentA;
}

void rightEncoderISR() {
  uint8_t currentA = digitalRead(RIGHT_ENCODER_A);
  uint8_t currentB = digitalRead(RIGHT_ENCODER_B);
  
  if (currentA != rightLastA) {
    rightEncoderCount += (currentA == currentB) ? 1 : -1;
  }
  rightLastA = currentA;
}

// =============================================================================
// UTILITY FUNCTIONS - INLINED FOR EFFICIENCY
// =============================================================================
inline float normalizeAngle(float angle) {
  while (angle > PI) angle -= TWO_PI;
  while (angle < -PI) angle += TWO_PI;
  return angle;
}

inline float radToDeg(float rad) { return rad * 57.2957795f; }
inline float degToRad(float deg) { return deg * 0.0174532925f; }

inline float sgn(float value) {
  return (value > 0) ? 1.0f : ((value < 0) ? -1.0f : 0.0f);
}

// =============================================================================
// MOTOR CONTROL - OPTIMIZED
// =============================================================================
void setMotorSpeed(uint8_t motor, int16_t speed) {
  speed = constrain(speed, -255, 255);
  
  uint8_t pwmPin = (motor == 0) ? LEFT_MOTOR_PWM : RIGHT_MOTOR_PWM;
  uint8_t dir1Pin = (motor == 0) ? LEFT_MOTOR_DIR1 : RIGHT_MOTOR_DIR1;
  uint8_t dir2Pin = (motor == 0) ? LEFT_MOTOR_DIR2 : RIGHT_MOTOR_DIR2;
  
  if (speed > 0) {
    digitalWrite(dir1Pin, HIGH);
    digitalWrite(dir2Pin, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(dir1Pin, LOW);
    digitalWrite(dir2Pin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void updateMotorControl() {
  uint32_t now = millis();
  float dt = (now - lastVelUpdate) * 0.001f;
  if (dt < 0.01f) return; // Max 100Hz update
  
  // Calculate velocities from encoders
  static long lastLeftCount = 0, lastRightCount = 0;
  long leftDelta = leftEncoderCount - lastLeftCount;
  long rightDelta = rightEncoderCount - lastRightCount;
  
  leftCurrentVel = (leftDelta * CM_PER_COUNT) / dt;
  rightCurrentVel = (rightDelta * CM_PER_COUNT) / dt;
  
  lastLeftCount = leftEncoderCount;
  lastRightCount = rightEncoderCount;
  lastVelUpdate = now;
  
  // PID control
  static int16_t leftPWM = 0, rightPWM = 0;
  leftPWM += (int16_t)leftMotorPID.update(leftTargetVel - leftCurrentVel);
  rightPWM += (int16_t)rightMotorPID.update(rightTargetVel - rightCurrentVel);
  
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);
  
  setMotorSpeed(0, leftPWM);
  setMotorSpeed(1, rightPWM);
}

inline void moveVelocity(float leftVel, float rightVel) {
  leftTargetVel = leftVel;
  rightTargetVel = rightVel;
}

inline void stopMotors() {
  moveVelocity(0, 0);
  delay(50);
  setMotorSpeed(0, 0);
  setMotorSpeed(1, 0);
}

// =============================================================================
// ODOMETRY - OPTIMIZED
// =============================================================================
void updateOdometry() {
  uint32_t now = millis();
  float dt = (now - lastOdometryUpdate) * 0.001f;
  if (dt < 0.01f) return;
  
  // Update heading from gyro
  float gyroZ = degToRad(imu.getGyroZ());
  gyroHeading += gyroZ * dt;
  gyroHeading = normalizeAngle(gyroHeading);
  
  // Update position from encoders
  static long lastLeftEncoder = 0, lastRightEncoder = 0;
  long leftDelta = leftEncoderCount - lastLeftEncoder;
  long rightDelta = rightEncoderCount - lastRightEncoder;
  
  float leftDist = leftDelta * CM_PER_COUNT;
  float rightDist = rightDelta * CM_PER_COUNT;
  float avgDist = (leftDist + rightDist) * 0.5f;
  
  robotPose.x += avgDist * cos(gyroHeading);
  robotPose.y += avgDist * sin(gyroHeading);
  robotPose.theta = gyroHeading;
  
  lastLeftEncoder = leftEncoderCount;
  lastRightEncoder = rightEncoderCount;
  lastOdometryUpdate = now;
}

// =============================================================================
// PATH GENERATION - MEMORY EFFICIENT
// =============================================================================
void convertToAbsoluteCoords() {
  for (uint8_t i = 0; i < pathPointCount; i++) {
    pathPoints[i].x = pathPoints[i].x * FIELD_SQUARE_SIZE + FIELD_SQUARE_SIZE * 0.5f;
    pathPoints[i].y = pathPoints[i].y * FIELD_SQUARE_SIZE + FIELD_SQUARE_SIZE * 0.5f;
    // pathPoints[i].theta remains as speed value
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
    segment->type = 0; // Path segment
    segment->count = 1;
    segment->points[0] = start;
    
    // Interpolate
    interpolatePath(segment->points, &segment->count, start, end, start.getSpeed());
    
    // Add end point
    if (segment->count < MAX_PATH_POINTS) {
      segment->points[segment->count] = end;
      segment->count++;
    }
    
    // Check for 180-degree turn
    if (i < pathPointCount - 2 && pathPoints[i + 2].equals(start)) {
      segmentCount++;
      
      // Add turn segment
      if (segmentCount < MAX_PATH_SEGMENTS) {
        PathSegment* turnSegment = &pathSegments[segmentCount];
        turnSegment->type = 1; // Turn segment
        turnSegment->data = normalizeAngle(start.angleTo(end) + PI);
        segmentCount++;
      }
    } else {
      segmentCount++;
    }
  }
}

// =============================================================================
// PURE PURSUIT - OPTIMIZED
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
  if (a < 0.0001f) return -1; // Points too close
  
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
    updateMotorControl();
    
    float error = normalizeAngle(targetHeading - robotPose.getHeading());
    if (abs(error) < tolerance) break;
    
    float turnSpeed = constrain(turnPID.update(error), -40, 40);
    moveVelocity(-turnSpeed, turnSpeed);
    delay(20);
  }
  
  stopMotors();
}

void followPath(const Position* path, uint8_t pathSize, float lookahead, uint32_t endTime) {
  while (millis() < endTime) {
    updateOdometry();
    updateMotorControl();
    
    uint8_t closest = findClosest(path, pathSize);
    
    // Check if at stop point
    if (path[closest].isStop()) break;
    
    Position lookaheadPoint = getLookaheadPoint(path, pathSize, closest, lookahead);
    float curvature = calculateCurvature(lookaheadPoint);
    
    // Speed control
    float targetSpeed = path[closest].getSpeed();
    float speedMultiplier = 1.0f / (1.0f + abs(curvature) * 20.0f);
    targetSpeed *= speedMultiplier;
    targetSpeed = max(targetSpeed, 5.0f);
    
    // Differential drive
    float leftSpeed = targetSpeed * (2 + curvature * DRIVE_TRACK_WIDTH) * 0.5f;
    float rightSpeed = targetSpeed * (2 - curvature * DRIVE_TRACK_WIDTH) * 0.5f;
    
    moveVelocity(leftSpeed, rightSpeed);
    delay(20);
  }
  
  stopMotors();
}

// =============================================================================
// MAIN FUNCTIONS
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

void setup() {
  Serial.begin(115200);
  Serial.println(F("Robot Tour - Optimized Version"));
  
  // Initialize pins
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(BEEPER_PIN, OUTPUT);
  
  // Motor pins
  const uint8_t motorPins[] = {LEFT_MOTOR_PWM, LEFT_MOTOR_DIR1, LEFT_MOTOR_DIR2,
                               RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR1, RIGHT_MOTOR_DIR2};
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
  }
  
  // Encoder pins
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  
  // Setup interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, CHANGE);
  
  // Startup beep
  digitalWrite(BEEPER_PIN, HIGH);
  delay(100);
  digitalWrite(BEEPER_PIN, LOW);
  
  // Initialize IMU
  if (!imu.begin()) {
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
  
  // Ready
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
  
  stopMotors();
  
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
  updateMotorControl();
  
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
    Serial.println(F("°)"));
  }
  
  delay(10);
}

