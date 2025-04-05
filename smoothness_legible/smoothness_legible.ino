#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

unsigned int counts00 = 102;
unsigned int counts60 = 240;
unsigned int counts90 = 307;
unsigned int counts120 = 375;
unsigned int counts180 = 512;

float currentPosition = counts00;
float maxSpeed = 5.0;       // Maximum speed
float acceleration = 0.01;  // Acceleration rate
float deceleration = 0.8;   // Deceleration rate
float currentSpeed = 0.0;   // Current speed

unsigned long previousMillis = 0;
const long interval = 10;

void setup() {
  Serial.begin(9600);
  servos.begin();
  servos.setPWMFreq(50);
  servos.setPWM(0, 0, counts90);
  servos.setPWM(1, 0, counts90);
  delay(1000);
}

void moveToPosition(int servon_num, float targetPosition) {
  while (abs(currentPosition - targetPosition) > 0.5) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      float distanceToTarget = targetPosition - currentPosition;
      float distanceRemaining = abs(distanceToTarget);

      float direction;
      if (distanceToTarget > 0) {
        direction = 1.0;
      } else {
        direction = -1.0;
      }

      float decelerationDistance = (currentSpeed * currentSpeed) / (2.0 * deceleration);

      if (distanceRemaining <= decelerationDistance) {
        if (currentSpeed - deceleration > 0.3) {
          currentSpeed = currentSpeed - deceleration;
        } else {
          currentSpeed = 0.3;
        }
      } else {
        if (currentSpeed + acceleration < maxSpeed) {
          currentSpeed = currentSpeed + acceleration;
        } else {
          currentSpeed = maxSpeed;
        }
      }

      if (distanceRemaining < 10) {
        float calculatedSpeed = distanceRemaining * 0.1 + 0.2;
        if (currentSpeed > calculatedSpeed) {
          currentSpeed = calculatedSpeed;
        }
      }

      currentPosition += direction * currentSpeed;
      servos.setPWM(servon_num, 0, (unsigned int)currentPosition);
    }
  }

  currentPosition = targetPosition;
  servos.setPWM(servon_num, 0, (unsigned int)currentPosition);
  currentSpeed = 0.0;
}

void naturalEyeMovement() {
  float randomTarget = random(counts00, counts120);
  moveToPosition(0, randomTarget);
  delay(random(500, 2000));
}

void blinkEye() {
  float currentPos = currentPosition;

  float savedMaxSpeed = maxSpeed;
  float savedAcceleration = acceleration;

  maxSpeed = 15.0;
  acceleration = 1.0;

  moveToPosition(0, counts00);
  delay(100);
  moveToPosition(0, currentPos);

  maxSpeed = savedMaxSpeed;
  acceleration = savedAcceleration;
}

void loop() {
  moveToPosition(0,counts180);
  delay(1000);
  moveToPosition(0,counts00);
}
