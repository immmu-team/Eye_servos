#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

// Calibración para el rango PWM de los servos
const unsigned int counts00 = 102;
const unsigned int counts90 = 307;
const unsigned int counts180 = 512;

// Estructuras para representar cada servo
struct ServoData {
  int servoNum;
  float currentPosition;
};

ServoData servoX = {0, (float)counts90};
ServoData servoY = {1, (float)counts90};

// Función smoothstep para suavizar la interpolación
float smoothstep(float t) {
  return t * t * (3 - 2 * t); // curva suave entre 0 y 1
}

// Función para generar un valor aleatorio dentro de un rango (inclusive)
float randomRange(float minValue, float maxValue) {
  return minValue + (random(0, 1000) / 1000.0) * (maxValue - minValue);
}

// Mueve ambos servos de forma sincronizada y suavizada
void moveEyeTo(float xTarget, float yTarget, unsigned long totalTimeMs, unsigned long stepTimeMs) {
  float xStart = servoX.currentPosition;
  float yStart = servoY.currentPosition;

  float dx = xTarget - xStart;
  float dy = yTarget - yStart;

  int steps = totalTimeMs / stepTimeMs;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;              // va de 0.0 a 1.0
    float easing = smoothstep(t);            // aplica suavizado

    servoX.currentPosition = xStart + dx * easing;
    servoY.currentPosition = yStart + dy * easing;

    servos.setPWM(servoX.servoNum, 0, (int)servoX.currentPosition);
    servos.setPWM(servoY.servoNum, 0, (int)servoY.currentPosition);

    vTaskDelay(pdMS_TO_TICKS(stepTimeMs));
  }
}

// Tarea que simula movimiento continuo del ojo con movimientos aleatorios
void eyeTask(void* parameter) {
  while (true) {
    // Generación de posiciones aleatorias dentro del rango (0 a 180 grados)
    float randomX = randomRange(0, 180);  // Rango de movimiento para el servo X
    float randomY = randomRange(0, 180);  // Rango de movimiento para el servo Y

    // Convertir las posiciones aleatorias a valores PWM (en este caso, se mapea de 0-180 a counts)
    float targetX = counts00 + (randomX / 180.0) * (counts180 - counts00);
    float targetY = counts00 + (randomY / 180.0) * (counts180 - counts00);

    // Mover los servos a las nuevas posiciones aleatorias con suavizado
    moveEyeTo(targetX, targetY, 1000, 20);  // 1 segundo para moverse, paso cada 20ms

    // Esperar un momento antes de moverlos nuevamente
    vTaskDelay(pdMS_TO_TICKS(1000));  // Pausa de 1 segundo entre movimientos
  }
}

void setup() {
  Serial.begin(115200);
  servos.begin();
  servos.setPWMFreq(50);

  // Posición inicial al centro
  servos.setPWM(servoX.servoNum, 0, counts90);
  servos.setPWM(servoY.servoNum, 0, counts90);

  xTaskCreate(eyeTask, "EyeTask", 4096, NULL, 1, NULL);
}

void loop() {
  // No se usa en FreeRTOS
}
