#ifndef POSITIONCONTROL_H
#define POSITIONCONTROL_H

#include <Arduino.h>
#include <Encoder.h>
#include <AutoPID.h>

class PositionControl
{

private:
  // Motor Pin Configuration
  uint8_t pwmPin;
  uint8_t dirPin;
  uint8_t encA;
  uint8_t encB;
  bool polarity_reverse = false;

  //Motor Related Parameters
  long CPR = 36124;
  float motorRadius = 1; // in metres
  float maxSpeed = 2.0; // Rotation Per Second

  // PID Control Related Parameter
  double outputMin  = -50;
  double outputMax = 50;
  float kP  = 0.1;
  float kI  = 0.03;
  float kD  = 0.02;


public: 
  //Velocity control related parameters
  double currentPosition;
  double setPoint = 0;
  double outputPWM= 0;
  double tolerance = 0.1;

  // double * position;
  AutoPID pid{&currentPosition, &setPoint, &outputPWM, outputMin, outputMax, kP, kI, kD};
  Encoder * encoder;
  
  void  initialSetup()
  {
  // Setting the pin configurations
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  digitalWrite(dirPin , LOW);
  analogWrite(pwmPin , 0);
  delay(2000);
  //variables for velocity calculation
  setPoint  = 0.0;
  currentPosition = 0;
  outputPWM  = 0;
  }

  PositionControl()
  {
    // params:  direction pin , pwm pin , encoder A , encoder B
   
  }

  void setPin(uint8_t pinDir , uint8_t pinPWM ,uint8_t aEnc , uint8_t bEnc )
  {
  dirPin = pinDir;
  pwmPin = pinPWM;
  encA  = aEnc;
  encB = bEnc;
  encoder  =  new Encoder(encA , encB);
  initialSetup();

  }

  ~PositionControl()
  {
    // destructor for the class
    delete encoder;
  }

  void setReversePolarity(bool polarity)
  {
    polarity_reverse = polarity;
  }

  void setTolerance(float delta)
  {
    tolerance = delta;
  }

  void setMotorPWM(int PWM)
  {
  if (polarity_reverse)
    PWM = PWM * -1;
    // Changing the motor direction based on the PWM value
  if (PWM < 0)
      digitalWrite(dirPin , LOW);
  else 
      digitalWrite(dirPin , HIGH);

  // generating the PWM pulses for changing the motor velocity
    analogWrite(pwmPin , abs(PWM));
  }

  void setPIDValue(float p , float i , float d )
  {
    kP = p;
    kI = i;
    kD = d;
    pid.setGains(kP, kI , kD);
  }

  void setPIDOutput(int min , int max)
  {
    outputMin = min;
    outputMax = max;
    pid.setOutputRange(outputMin , outputMax);
  }

  void setCPR(int cpr)
  {
    CPR = cpr;
    Serial.println("CPR");
    Serial.println(CPR);
  }

  void setMotorMaxSpeed(float speed)
  {
    maxSpeed = speed;
  }

  void setAngle(float theta)
  {
    // input: motor speed in rotation per second
    setPoint = long((theta* CPR)/360);
  }

  float getAngle()
  {
    return ((encoder->read()* 360)/double(CPR));
  }

  void controlLoop()
  { 
  currentPosition = encoder->read();
  pid.run();
  if (setPoint <0)

        if ((setPoint * (1.0 - tolerance) > currentPosition) && (setPoint * (1 + tolerance) < currentPosition))
        {
          setMotorPWM( 0);
          // Serial.println("Reached the target");
        }
      else
        {
          setMotorPWM(outputPWM);
        }

  else

      if ((setPoint * (1.0 - tolerance) < currentPosition) && (setPoint * (1 + tolerance) > currentPosition))
        {
          setMotorPWM( 0);
          // Serial.println("Reached the target");
        }
      else
        {
          setMotorPWM(outputPWM);
        }
  
  }


};

#endif
