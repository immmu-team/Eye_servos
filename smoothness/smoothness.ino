#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

// Servo position calibration
unsigned int counts00 = 102;
unsigned int counts60 = 240;
unsigned int counts90 = 307;
unsigned int counts120 = 375;
unsigned int counts180 = 512;

// Current position tracking
float currentPosition = counts00;

// Acceleration and smoothing parameters
float maxSpeed = 2.0;       // Maximum speed
float acceleration = 0.05;  // Acceleration rate
float deceleration = 0.2;   // Deceleration rate
float currentSpeed = 0.0;   // Current speed

// Timing control for natural movement
unsigned long previousMillis = 0;
const long interval = 10;   // Update interval in milliseconds

void setup() {
  Serial.begin(9600);
  servos.begin();  
  servos.setPWMFreq(50); // PWM frequency of 50Hz (T=20ms)
  
  // Initialize position
  servos.setPWM(0, 0, counts90);
  servos.setPWM(1, 0, counts90);  
  delay(1000);
}

// Function for smooth movement to target position
void moveToPosition(int servon_num, float targetPosition) {
  // Continue until we're close enough to the target
  while (abs(currentPosition - targetPosition) > 0.5) {
    unsigned long currentMillis = millis();
    
    // Only update at the specified interval for smooth control
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      
      // Calculate distance to target
      float distanceToTarget = targetPosition - currentPosition;
      float distanceRemaining = abs(distanceToTarget);
      
      // Determine direction of movement
      float direction = (distanceToTarget > 0) ? 1.0 : -1.0;
      
      // Calculate deceleration distance based on current speed
      float decelerationDistance = (currentSpeed * currentSpeed) / (2.0 * deceleration);
      
      // Adjust speed based on distance remaining
      if (distanceRemaining <= decelerationDistance) {
        // We need to start slowing down
        currentSpeed = (currentSpeed - deceleration > 0.3) ? (currentSpeed - deceleration) : 0.3;
      } else {
        // We can accelerate or maintain speed
        currentSpeed = (currentSpeed + acceleration < maxSpeed) ? (currentSpeed + acceleration) : maxSpeed;
      }
      
      // Apply easing at the start and end of movements
      if (distanceRemaining < 10) {
        // Slow down more dramatically near target
        float calculatedSpeed = distanceRemaining * 0.1 + 0.2;
        currentSpeed = (currentSpeed < calculatedSpeed) ? currentSpeed : calculatedSpeed;
      }
      
      // Update position
      currentPosition += direction * currentSpeed;
      
      // Apply position to servo
      servos.setPWM(servon_num, 0, (unsigned int)currentPosition);
      
      // Debug info
      // Serial.print("Target: "); Serial.print(targetPosition);
      // Serial.print(" Current: "); Serial.print(currentPosition);
      // Serial.print(" Speed: "); Serial.println(currentSpeed);
    }
  }
  
  // Ensure we reach the exact target position
  currentPosition = targetPosition;
  servos.setPWM(servon_num, 0, (unsigned int)currentPosition);
  currentSpeed = 0.0;
}

// Function for random natural eye movements
void naturalEyeMovement() {
  // Generate random position within range
  float randomTarget = random(counts00, counts120);
  
  // Move to the random position
  moveToPosition(randomTarget);
  
  // Random pause to simulate natural eye fixation
  delay(random(500, 2000));
}

// Function for simulating a blink
void blinkEye() {
  float currentPos = currentPosition;
  
  // Quick movement to closed position
  float savedMaxSpeed = maxSpeed;
  float savedAcceleration = acceleration;
  
  // Faster parameters for blinking
  maxSpeed = 15.0;
  acceleration = 1.0;
  
  // Close eye (move to bottom position)
  moveToPosition(counts00);
  
  // Pause briefly while closed
  delay(100);
  
  // Open eye (return to previous position)
  moveToPosition(currentPos);
  
  // Restore original parameters
  maxSpeed = savedMaxSpeed;
  acceleration = savedAcceleration;
}

void loop() {
  // Left to right full sweep with smooth acceleration/deceleration
  moveToPosition(0,counts120);
  delay(1000);
  
  moveToPosition(1, counts120);
  delay(1000);
  
  // Demonstrate natural eye movements
  //for (int i = 0; i < 5; i++) {
  //  naturalEyeMovement();
  //}
  
  // Demonstrate blinking
  //blinkEye();
  //delay(2000);
}