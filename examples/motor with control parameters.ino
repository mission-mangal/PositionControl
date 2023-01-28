#include "PositionControl.h"
#include <Arduino.h>

MotorControl motor1;

#define dirPin 9
#define pwmPin 8
#define encA 6
#define encB 7
#define p 50
#define i 30
#define d 25
#define outputMin
#define outputMax
#define CPR 3000
#define maxSpeed 2

void setup()
{
// initial microcontroller health indication
Serial.begin(9600);

//setting motor pin parameters
motor1.setPin(4, 5, 2, 3);
motor1.setReversePolarity(false);
motor1.setTolerance(0.05);
motor1.setPIDValue(p , i , d );
motor1.setPIDOutput(outputMin ,outputMax);
motor1.setCPR(CPR);
motor1.setAngle(90);

}


void loop()
{
motor1.controlLoop();
Serial.println(motor1.getAngle());
Serial.println(motor1.currentPosition);
}