#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

// Calibración del servo
const unsigned int counts00 = 102;
const unsigned int counts90 = 307;
const unsigned int counts180 = 512;

// Estructura de servo
struct ServoData {
  int servoNum;
  float currentPosition;
};

ServoData servoX = {0, (float)counts90};
ServoData servoY = {1, (float)counts90};

// Suavizado tipo smoothstep
float smoothstep(float t) {
  return t * t * (3 - 2 * t);
}

// Mapea de grados a PWM
float degreesToCounts(float angleDegrees) {
  return counts00 + (angleDegrees / 180.0) * (counts180 - counts00);
}

// Mueve los servos suavemente
void moveEyeTo(float xTarget, float yTarget, unsigned long totalTimeMs, unsigned long stepTimeMs) {
  float xStart = servoX.currentPosition;
  float yStart = servoY.currentPosition;

  float dx = xTarget - xStart;
  float dy = yTarget - yStart;

  int steps = totalTimeMs / stepTimeMs;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float easing = smoothstep(t);

    servoX.currentPosition = xStart + dx * easing;
    servoY.currentPosition = yStart + dy * easing;

    servos.setPWM(servoX.servoNum, 0, (int)servoX.currentPosition);
    servos.setPWM(servoY.servoNum, 0, (int)servoY.currentPosition);

    vTaskDelay(pdMS_TO_TICKS(stepTimeMs));
  }
}

// Estructura para cada instrucción
struct MovementInstruction {
  float angleX;
  float angleY;
  unsigned long duration; // duración del movimiento en ms
  unsigned long pause;    // pausa después del movimiento en ms
};

// Rutina de movimientos (puedes modificar esta lista)
MovementInstruction routine[] = {
  {90, 90, 1000, 500},   // Centro
  {120, 60, 800, 500},   // Arriba derecha
  {60, 60, 800, 500},    // Arriba izquierda
  {60, 120, 800, 500},   // Abajo izquierda
  {120, 120, 800, 500},  // Abajo derecha
  {90, 90, 1000, 1000},  // Vuelve al centro
};

const int routineLength = sizeof(routine) / sizeof(routine[0]);

// Ejecuta la rutina
void eyeTask(void* parameter) {
  while (true) {
    for (int i = 0; i < routineLength; i++) {
      MovementInstruction step = routine[i];

      float targetX = degreesToCounts(step.angleX);
      float targetY = degreesToCounts(step.angleY);

      moveEyeTo(targetX, targetY, step.duration, 20);  // paso cada 20 ms
      vTaskDelay(pdMS_TO_TICKS(step.pause));           // pausa después del movimiento
    }
  }
}

void setup() {
  Serial.begin(115200);
  servos.begin();
  servos.setPWMFreq(50);

  // Posición inicial al centro
  servos.setPWM(servoX.servoNum, 0, counts90);
  servos.setPWM(servoY.servoNum, 0, counts90);

  // Inicia la rutina
  xTaskCreate(eyeTask, "EyeTask", 4096, NULL, 1, NULL);
}

void loop() {
  // No se usa en FreeRTOS
}
