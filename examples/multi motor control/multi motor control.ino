#include "PositionControl.h"
#include <Arduino.h>

PositionControl frontLeft;
PositionControl frontRight;


void setup()
{
// initial microcontroller health indication
Serial.begin(9600);
//setting motor pin parameters
frontLeft.setPin(4, 5, 2, 3);
frontLeft.setReversePolarity(false);
frontLeft.setTolerance(0.05);
frontLeft.setAngle(90);
frontRight.setPin(9, 8, 6, 7);
frontRight.setReversePolarity(true);
frontRight.setTolerance(0.05);
frontRight.setAngle(90);
}

void loop()
{
frontLeft.controlLoop();
Serial.println(frontLeft.getAngle());
Serial.println(frontLeft.currentPosition);
frontRight.controlLoop();
Serial.println(frontRight.getAngle());
Serial.println(frontRight.currentPosition);
}

