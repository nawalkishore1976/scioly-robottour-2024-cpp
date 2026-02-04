#include <Wire.h>
#include <math.h>

// =============================================================================
// CONFIGURATION
// =============================================================================
#define MAX_PATH_POINTS 200
#define MAX_PATH_SEGMENTS 50
#define DRIVE_TRACK_WIDTH 15.0  // inches between wheels
#define WHEEL_DIAMETER 6.5      // inches
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
#define START_BUTTON_PIN 9
#define LIGHT_PIN 13
#define BEEPER_PIN 12

// =============================================================================
// DATA STRUCTURES
// =============================================================================
struct Position {
  float x;
  float y;
  float theta;
  
  float distance(const Position& other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    return sqrt(dx * dx + dy * dy);
  }
  
  Position lerp(const Position& other, float t) const {
    return {x + t * (other.x - x), y + t * (other.y - y), theta};
  }
  
  float angle(const Position& other) const {
    return atan2(other.y - y, other.x - x);
  }
  
  bool equals(const Position& other) const {
    return (abs(x - other.x) < 0.1) && (abs(y - other.y) < 0.1);
  }
};

struct PathSegment {
  bool isTurn;
  float turnAngle;
  Position points[MAX_PATH_POINTS];
  int pointCount;
};

// =============================================================================
// DUMMY HARDWARE CLASSES
// =============================================================================
class RB_Motor {
public:
  RB_Motor(int pwmPin, int dir1Pin, int dir2Pin) 
    : pwmPin(pwmPin), dir1Pin(dir1Pin), dir2Pin(dir2Pin) {
    pinMode(pwmPin, OUTPUT);
    pinMode(dir1Pin, OUTPUT);
    pinMode(dir2Pin, OUTPUT);
  }
  
  void setSpeed(int speed) {  // -255 to 255
    if (speed > 0) {
      digitalWrite(dir1Pin, HIGH);
      digitalWrite(dir2Pin, LOW);
      analogWrite(pwmPin, constrain(speed, 0, 255));
    } else if (speed < 0) {
      digitalWrite(dir1Pin, LOW);
      digitalWrite(dir2Pin, HIGH);
      analogWrite(pwmPin, constrain(-speed, 0, 255));
    } else {
      digitalWrite(dir1Pin, LOW);
      digitalWrite(dir2Pin, LOW);
      analogWrite(pwmPin, 0);
    }
  }
  
  void stop() {
    setSpeed(0);
  }
  
  long getEncoder() {
    // Dummy encoder reading - replace with actual encoder reading
    return encoderCount;
  }
  
private:
  int pwmPin, dir1Pin, dir2Pin;
  long encoderCount = 0;  // Replace with actual encoder integration
};

class ICM20498_Helper {
public:
  bool begin() {
    Wire.begin();
    // Initialize ICM-20498 - dummy implementation
    return true;
  }
  
  float getHeading() {
    // Return current heading in radians - replace with actual IMU reading
    return currentHeading;
  }
  
  void calibrate() {
    // Calibration routine - dummy implementation
    currentHeading = 0;
  }
  
private:
  float currentHeading = 0;
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
RB_Motor leftMotor(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR1, LEFT_MOTOR_DIR2);
RB_Motor rightMotor(RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR1, RIGHT_MOTOR_DIR2);
ICM20498_Helper imu;

Position currentPose = {0, 0, 0};
Position lastPose = {0, 0, 0};

// Fixed path points - convert to absolute coordinates
Position pathPoints[] = {
  {0, 0, 0}, {1, 0, 0}, {0, 0, 0}, {0, 1, 0},
  {1, 1, 0}, {0, 1, 0}, {0, 2, 0}, {1, 2, 0},
  {1, 3, 0}, {3, 3, 0}, {3, 2, 0}, {2 + FINISH_OFFSET, 2, 0}
};
int pathPointCount = sizeof(pathPoints) / sizeof(pathPoints[0]);

PathSegment pathSegments[MAX_PATH_SEGMENTS];
int segmentCount = 0;

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================
float sgn(float value) {
  return (value > 0) ? 1.0 : ((value < 0) ? -1.0 : 0.0);
}

float radToDeg(float rad) {
  return rad * 180.0 / PI;
}

float degToRad(float deg) {
  return deg * PI / 180.0;
}

float slew(float target, float current, float maxChange) {
  float diff = target - current;
  if (abs(diff) <= maxChange) {
    return target;
  }
  return current + sgn(diff) * maxChange;
}

// =============================================================================
// ODOMETRY FUNCTIONS
// =============================================================================
void updateOdometry() {
  // Simple odometry using encoders and IMU
  // This is a simplified version - implement proper odometry
  float heading = imu.getHeading();
  
  // Get encoder readings (dummy values for now)
  long leftEncoder = leftMotor.getEncoder();
  long rightEncoder = rightMotor.getEncoder();
  
  // Update position based on encoder readings and heading
  // This is a placeholder - implement actual odometry calculations
  currentPose.theta = heading;
}

void setPose(float x, float y, float theta) {
  currentPose.x = x;
  currentPose.y = y;
  currentPose.theta = theta;
}

// =============================================================================
// PATH GENERATION FUNCTIONS
// =============================================================================
void toAbsoluteCoordinates() {
  for (int i = 0; i < pathPointCount; i++) {
    pathPoints[i].x = 50 * pathPoints[i].x + 25;
    pathPoints[i].y = 50 * pathPoints[i].y + 25;
  }
}

void interpolatePath(Position* path, int* pathSize, const Position& start, const Position& end) {
  double d = start.distance(end);
  
  for (double n = 1; n < d && *pathSize < MAX_PATH_POINTS - 1; n++) {
    path[*pathSize] = {
      start.x + n / d * (end.x - start.x),
      start.y + n / d * (end.y - start.y),
      50  // speed
    };
    (*pathSize)++;
  }
}

void generatePath() {
  segmentCount = 0;
  
  for (int i = 0; i < pathPointCount - 1 && segmentCount < MAX_PATH_SEGMENTS; i++) {
    Position start = pathPoints[i];
    Position end = pathPoints[i + 1];
    
    PathSegment* segment = &pathSegments[segmentCount];
    segment->isTurn = false;
    segment->pointCount = 0;
    
    // Interpolate between current and next
    interpolatePath(segment->points, &segment->pointCount, start, end);
    
    // Add end point
    segment->points[segment->pointCount] = {
      end.x, end.y, 
      (i + 1 == pathPointCount - 1) ? 0 : 50  // speed
    };
    segment->pointCount++;
    
    // Check if 180deg turn
    if (i < pathPointCount - 2 && pathPoints[i + 2].equals(start)) {
      segment->points[segment->pointCount - 1].theta = 0;  // stop
      segmentCount++;
      
      // Add turn segment
      if (segmentCount < MAX_PATH_SEGMENTS) {
        PathSegment* turnSegment = &pathSegments[segmentCount];
        turnSegment->isTurn = true;
        turnSegment->turnAngle = start.angle(end);
        segmentCount++;
      }
    } else {
      segmentCount++;
    }
  }
}

// =============================================================================
// PURE PURSUIT FUNCTIONS
// =============================================================================
int findClosest(Position pose, Position* path, int pathSize) {
  int closestPoint = 0;
  float closestDist = 1000000;
  
  for (int i = 0; i < pathSize; i++) {
    float dist = pose.distance(path[i]);
    if (dist < closestDist) {
      closestDist = dist;
      closestPoint = i;
    }
  }
  
  return closestPoint;
}

float circleIntersect(const Position& p1, const Position& p2, 
                     const Position& pose, float lookaheadDist) {
  Position d = {p2.x - p1.x, p2.y - p1.y, 0};
  Position f = {p1.x - pose.x, p1.y - pose.y, 0};
  
  float a = d.x * d.x + d.y * d.y;
  float b = 2 * (f.x * d.x + f.y * d.y);
  float c = (f.x * f.x + f.y * f.y) - lookaheadDist * lookaheadDist;
  float discriminant = b * b - 4 * a * c;
  
  if (discriminant >= 0) {
    discriminant = sqrt(discriminant);
    float t1 = (-b - discriminant) / (2 * a);
    float t2 = (-b + discriminant) / (2 * a);
    
    if (t2 >= 0 && t2 <= 1) return t2;
    else if (t1 >= 0 && t1 <= 1) return t1;
  }
  
  return -1;
}

Position lookaheadPoint(Position* path, int pathSize, int closest, float lookaheadDist) {
  for (int i = closest; i < pathSize - 1; i++) {
    Position lastPathPose = path[i];
    Position currentPathPose = path[i + 1];
    
    float t = circleIntersect(lastPathPose, currentPathPose, currentPose, lookaheadDist);
    
    if (t != -1) {
      return lastPathPose.lerp(currentPathPose, t);
    }
  }
  
  return path[pathSize - 1];  // Default to end point
}

float findLookaheadCurvature(const Position& pose, float heading, const Position& lookahead) {
  float side = sgn(sin(heading) * (lookahead.x - pose.x) - 
                   cos(heading) * (lookahead.y - pose.y));
  float a = -tan(heading);
  float c = tan(heading) * pose.x - pose.y;
  float x = abs(a * lookahead.x + lookahead.y + c) / sqrt((a * a) + 1);
  float d = sqrt((lookahead.x - pose.x) * (lookahead.x - pose.x) + 
                 (lookahead.y - pose.y) * (lookahead.y - pose.y));
  
  return side * ((2 * x) / (d * d));
}

// =============================================================================
// MOTOR CONTROL FUNCTIONS
// =============================================================================
void moveVelocity(float leftVel, float rightVel) {
  // Convert velocity to motor speeds (-255 to 255)
  int leftSpeed = constrain(leftVel * 2, -255, 255);
  int rightSpeed = constrain(rightVel * 2, -255, 255);
  
  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);
}

void turnTo(float targetHeading) {
  float currentHeading = imu.getHeading();
  float error = targetHeading - currentHeading;
  
  // Normalize error to [-PI, PI]
  while (error > PI) error -= 2 * PI;
  while (error < -PI) error += 2 * PI;
  
  // Simple PID for turning
  float kP = 100.0;
  float tolerance = 0.1;
  
  while (abs(error) > tolerance) {
    currentHeading = imu.getHeading();
    error = targetHeading - currentHeading;
    
    while (error > PI) error -= 2 * PI;
    while (error < -PI) error += 2 * PI;
    
    float turnSpeed = constrain(error * kP, -150, 150);
    moveVelocity(-turnSpeed, turnSpeed);
    
    delay(20);
  }
  
  moveVelocity(0, 0);
}

void followPath(Position* path, int pathSize, float lookahead, unsigned long endTime) {
  Position lookaheadPose;
  float curvature;
  float targetVel;
  float prevVel = 0;
  int closestPoint;
  
  while (millis() < endTime) {
    updateOdometry();
    
    // Find closest point
    closestPoint = findClosest(currentPose, path, pathSize);
    
    // Check if at end
    if (path[closestPoint].theta == 0) break;
    
    // Find lookahead point
    lookaheadPose = lookaheadPoint(path, pathSize, closestPoint, lookahead);
    
    // Calculate curvature
    float curvatureHeading = PI/2 - currentPose.theta;
    curvature = findLookaheadCurvature(currentPose, curvatureHeading, lookaheadPose);
    
    // Calculate target velocity
    float timeRemaining = (endTime - millis()) / 1000.0;
    targetVel = 100;  // Simplified - use constant speed
    
    if (timeRemaining <= 0) targetVel = 150;  // Max speed if time is up
    targetVel = slew(targetVel, prevVel, 4);
    
    if (targetVel < 10) targetVel = 10;  // Prevent stalling
    prevVel = targetVel;
    
    // Calculate left and right velocities
    float targetLeftVel = targetVel * (2 + curvature * DRIVE_TRACK_WIDTH) / 2;
    float targetRightVel = targetVel * (2 - curvature * DRIVE_TRACK_WIDTH) / 2;
    
    // Ratio speeds to respect max speed
    float maxSpeed = max(abs(targetLeftVel), abs(targetRightVel));
    if (maxSpeed > 127) {
      targetLeftVel = targetLeftVel * 127 / maxSpeed;
      targetRightVel = targetRightVel * 127 / maxSpeed;
    }
    
    moveVelocity(targetLeftVel, targetRightVel);
    delay(20);
  }
  
  moveVelocity(0, 0);
}

// =============================================================================
// MAIN FUNCTIONS
// =============================================================================
int waitForButton() {
  int clicks = 0;
  bool lastState = digitalRead(START_BUTTON_PIN);
  unsigned long lastChange = millis();
  
  while (clicks == 0 || millis() - lastChange < 2000) {
    bool currentState = digitalRead(START_BUTTON_PIN);
    
    if (currentState != lastState && millis() - lastChange > 50) {
      if (!currentState) {  // Button pressed (assuming pull-up)
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
  
  // Initialize pins
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(BEEPER_PIN, OUTPUT);
  
  // Beep once
  digitalWrite(BEEPER_PIN, HIGH);
  delay(50);
  digitalWrite(BEEPER_PIN, LOW);
  
  // Initialize IMU
  if (!imu.begin()) {
    Serial.println("IMU initialization failed!");
    while (1);
  }
  
  // Set initial position
  Position startPosition = {50 * START_QUAD + 25, -14, 0};
  setPose(startPosition.x, startPosition.y, startPosition.theta);
  
  Serial.println("Robot initialized");
  
  // Generate path
  toAbsoluteCoordinates();
  generatePath();
  
  Serial.print("Generated ");
  Serial.print(segmentCount);
  Serial.println(" path segments");
  
  // Wait for button press
  digitalWrite(BEEPER_PIN, HIGH);
  delay(200);
  digitalWrite(BEEPER_PIN, LOW);
  
  int clicks = waitForButton();
  Serial.print("Button clicked ");
  Serial.print(clicks);
  Serial.println(" times");
  
  digitalWrite(LIGHT_PIN, LOW);
  
  // Calculate end time
  unsigned long endTime = millis() + TARGET_SECONDS * 1000;
  
  // Execute path
  for (int i = 0; i < segmentCount; i++) {
    PathSegment* segment = &pathSegments[i];
    
    if (segment->isTurn) {
      Serial.print("Turning to ");
      Serial.println(radToDeg(segment->turnAngle));
      turnTo(segment->turnAngle);
    } else {
      Serial.print("Following path segment ");
      Serial.print(i);
      Serial.print(" with ");
      Serial.print(segment->pointCount);
      Serial.println(" points");
      
      followPath(segment->points, segment->pointCount, 10, endTime);
    }
  }
  
  // Stop motors
  leftMotor.stop();
  rightMotor.stop();
  
  Serial.println("Path complete!");
}

void loop() {
  // Blink LED to indicate completion
  digitalWrite(LIGHT_PIN, HIGH);
  delay(1000);
  digitalWrite(LIGHT_PIN, LOW);
  delay(1000);
}