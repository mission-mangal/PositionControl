#include "PositionControl.h"
#include <Arduino.h>

PositionControl frontLeft;

#define dirPin 9
#define pwmPin 8
#define encA 6
#define encB 7


void setup()
{
// initial microcontroller health indication
Serial.begin(9600);

//setting motor pin parameters
frontLeft.setPin(4, 5, 2, 3);
frontLeft.setReversePolarity(false);
//Tolerance for error in control, have tolerance more than 0.01, otherwise motor inertia takes more time to reach the position with multiple oscillation
frontLeft.setTolerance(0.05);
frontLeft.setAngle(90);

}

void loop()
{
frontLeft.controlLoop();
Serial.println(frontLeft.getAngle());
Serial.println(frontLeft.currentPosition);
}

