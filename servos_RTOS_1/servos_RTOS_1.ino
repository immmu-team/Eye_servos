#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h> // Asegúrate de tener esta línea al inicio


Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

// random
float step_acc = 0.01;
float step_speed = 0.1;

float minVal_acc = 0.01;
float minVal_maxSpeed = 1.0;

float maxVal_acc = 0.1;
float maxVal_maxSpeed = 5.0;

long stepsCount_acc = (maxVal_acc - minVal_acc)/step_acc;
long stepsCount_maxSpeed = (maxVal_maxSpeed - minVal_maxSpeed)/step_speed;

// Calibración
unsigned int counts00 = 102;
unsigned int counts60 = 240;
unsigned int counts90 = 307;
unsigned int counts120 = 375;
unsigned int counts180 = 512;

struct ServoData {
  int servoNum;
  float targetPosition;
  float currentPosition;
  float currentSpeed;

  float maxSpeed;       // <-- ahora es por-servo
  float acceleration;   // <-- ahora es por-servo
  float deceleration;   // <-- también
};


float maxSpeed = 5.0;       // Maximum speed
float acceleration = 0.01;  // Acceleration rate
float deceleration = 0.1;   // Deceleration rate
float currentSpeed = 0.0;   // Current speed

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
      float decelerationDistance = (servo->currentSpeed * servo->currentSpeed) / (2.0 * deceleration);

      if (distanceRemaining <= decelerationDistance) {
        servo->currentSpeed = fmaxf(servo->currentSpeed - deceleration, 0.3);
      } else {
        servo->currentSpeed = fminf(servo->currentSpeed + acceleration, maxSpeed);
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

// Tarea de control para cada servo
void servoTask(void* parameter) {
  
  ServoData* servo = (ServoData*)parameter;
  randomSeed(analogRead(A0));

  while (true) {
    // Movimiento cíclico entre 90° y 120°

    maxSpeed = minVal_maxSpeed + (random(0, stepsCount_maxSpeed + 1) * step_speed);
    acceleration = minVal_acc + (random(0, stepsCount_acc + 1) * step_acc);
    Serial.println("maxspeed: " + String(maxSpeed) + " servoNum: " + String(servo->servoNum));

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
  Serial.begin(9600);


  // Crear y lanzar tareas para cada servo
  for (int i = 0; i < 2; i++) {
    ServoData* servo = new ServoData{ i, counts90, (float)counts90, 0.0 };
    servos.setPWM(i, 0, counts90);  // Posición inicial
    xTaskCreate(servoTask, "ServoTask", 2048, servo, 1, NULL);
  }
}

void loop() {
  // Nada en loop, el trabajo lo hace RTOS
}
