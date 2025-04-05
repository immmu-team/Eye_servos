#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define DELAY 2

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

unsigned int counts00 = 102;
unsigned int counts90 = 307;
unsigned int counts60 = 240;
unsigned int counts120 = 375;

void setup() {
  servos.begin();  
  servos.setPWMFreq(40); //Frecuecia PWM de 50Hz o T=20ms
  servos.setPWM(0,0,counts90);
  servos.setPWM(1,0,counts90);
  delay(5000);
}

void loop(){
  for(int duty = counts90; duty < counts120; duty=duty+1){
    for(int n=0; n<2; n++){
      servos.setPWM(n,0,duty);
    }
    delay(DELAY);  
  }
  delay(5000);

  for(int duty = counts120; duty > counts60; duty=duty-1){
    for(int n=0;n<2;n++){
      servos.setPWM(n,0,duty);
    }
    delay(DELAY);  
  }
  delay(5000);

  for(int duty = counts60; duty > counts90; duty=duty+1){
    for(int n=0;n<2;n++){
      servos.setPWM(n,0,duty);
    }
    delay(DELAY);  
  }
  delay(5000);
}
