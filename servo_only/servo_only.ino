#include <ESP32Servo.h>

Servo miServo;

const int pinServo = 16; // Pin GPIO al que está conectado el servo

void setup(){
  miServo.attach(pinServo);
}

void loop() 
{
  miServo.write(0);    // Mover el servo a 0 grados
  delay(1000);         // Esperar 1 segundo
  miServo.write(90);   // Mover el servo a 90 grados
  delay(1000);         // Esperar 1 segundo
  miServo.write(180);  // Mover el servo a 180 grados
  delay(1000);         // Esperar 1 segundo
}