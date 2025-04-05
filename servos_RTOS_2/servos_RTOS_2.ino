#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

// Parámetros de generación aleatoria
float step_acc = 0.01;
float step_speed = 0.1;

float minVal_acc = 0.01;
float minVal_maxSpeed = 1.0;

float maxVal_acc = 0.1;
float maxVal_maxSpeed = 5.0;

long stepsCount_acc = (maxVal_acc - minVal_acc)/step_acc;
long stepsCount_maxSpeed = (maxVal_maxSpeed - minVal_maxSpeed)/step_speed;

// Calibración de PWM para diferentes ángulos
unsigned int counts00 = 102;
unsigned int counts60 = 240;
unsigned int counts90 = 307;
unsigned int counts120 = 375;
unsigned int counts180 = 512;

// Estructura por servo
struct ServoData {
  int servoNum;
  float targetPosition;
  float currentPosition;
  float currentSpeed;
  float maxSpeed;
  float acceleration;
  float deceleration;
};

const long interval = 10;

void moveToPosition(ServoData* servo) {
  unsigned long previousMillis = millis();

  while (abs(servo->currentPosition - servo->targetPosition) > 0.5) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      float distanceToTarget = servo->targetPosition - servo->currentPosition;
      float distanceRemaining = abs(distanceToTarget);
      float direction = (distanceToTarget > 0) ? 1.0 : -1.0;
      float decelerationDistance = (servo->currentSpeed * servo->currentSpeed) / (2.0 * servo->deceleration);

      if (distanceRemaining <= decelerationDistance) {
        servo->currentSpeed = fmaxf(servo->currentSpeed - servo->deceleration, 0.3);
      } else {
        servo->currentSpeed = fminf(servo->currentSpeed + servo->acceleration, servo->maxSpeed);
      }

      if (distanceRemaining < 10) {
        float calculatedSpeed = distanceRemaining * 0.1 + 0.2;
        servo->currentSpeed = min(servo->currentSpeed, calculatedSpeed);
      }

      servo->currentPosition += direction * servo->currentSpeed;
      servos.setPWM(servo->servoNum, 0, (unsigned int)servo->currentPosition);
    }
  }

  servo->currentPosition = servo->targetPosition;
  servos.setPWM(servo->servoNum, 0, (unsigned int)servo->targetPosition);
  servo->currentSpeed = 0.0;
}

void servoTask(void* parameter) {
  ServoData* servo = (ServoData*)parameter;
  randomSeed(analogRead(A0));

  while (true) {
    // Valores aleatorios únicos para este servo
    servo->maxSpeed = minVal_maxSpeed + (random(0, stepsCount_maxSpeed + 1) * step_speed);
    servo->acceleration = minVal_acc + (random(0, stepsCount_acc + 1) * step_acc);
    servo->deceleration = 0.1; // Fijo por ahora, pero también podrías hacerlo aleatorio

    Serial.println("Servo " + String(servo->servoNum) +
                   " | maxSpeed: " + String(servo->maxSpeed) +
                   " | acceleration: " + String(servo->acceleration));

    servo->targetPosition = counts180;
    moveToPosition(servo);
    vTaskDelay(pdMS_TO_TICKS(1000));

    servo->targetPosition = counts00;
    moveToPosition(servo);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  servos.begin();
  servos.setPWMFreq(50);

  for (int i = 0; i < 2; i++) {
    ServoData* servo = new ServoData{
      i,
      counts90,               // targetPosition inicial
      (float)counts90,        // currentPosition inicial
      0.0,                    // currentSpeed inicial
      0.0,                    // maxSpeed se calcula después
      0.0,                    // acceleration se calcula después
      0.1                     // deceleration por defecto
    };

    servos.setPWM(i, 0, counts90);  // Posición inicial
    xTaskCreate(servoTask, "ServoTask", 2048, servo, 1, NULL);
  }
}

void loop() {
  // Nada aquí. Todo lo maneja el RTOS.
}
