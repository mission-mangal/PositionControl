#include "PositionControl.h"
#include <Arduino.h>


#include <ros.h>
#include <std_msgs/Float32.h>



PositionControl frontLeft;

#define dirPin 9
#define pwmPin 8
#define encA 6
#define encB 7


int RATE = 50;
float theta=0;
ros::NodeHandle nh;
std_msgs::Float32 position_angle;
std_msgs::Float32 position_ticks;
std_msgs::Float32 output_pwm;

double *setpoint=&frontLeft.setPoint;

void messageCb(const std_msgs::Float32 &state) {

  theta = (float)state.data;
  *setpoint = long((theta* frontLeft.CPR)/360);
  nh.loginfo("data received");
  
}
ros::Publisher position_angle_pub("position_angle", &position_angle);
ros::Publisher position_ticks_pub("position_ticks", &position_ticks);
ros::Publisher output_pwm_pub("output_pwm", &output_pwm);
ros::Subscriber<std_msgs::Float32>
  position_sub("position_in", &messageCb);


void setup() {

  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(position_sub);

  nh.advertise(position_angle_pub);
  nh.advertise(position_ticks_pub);
  nh.advertise(output_pwm_pub);

  
  // initial microcontroller health indication
  Serial1.begin(9600);

  //setting motor pin parameters
  frontLeft.setPin(12, 5, 2, 3);
  frontLeft.setReversePolarity(false);
  //Tolerance for error in control, have tolerance more than 0.01, otherwise motor inertia takes more time to reach the position with multiple oscillation
  frontLeft.setTolerance(0.001);
  // delay(5000);
  frontLeft.setCPR(80548);
  // delay(5000);
  // frontLeft.setPIDOutput(30, 30);
  // frontLeft.setAngle(90);
  // frontLeft.setAngle(-90);
}  

void loop() {
  frontLeft.controlLoop();
  if (nh.connected()) {
    position_angle.data=frontLeft.getAngle();
    position_ticks.data=frontLeft.currentPosition;
    output_pwm.data=frontLeft.outputPWM;

    position_angle_pub.publish(&position_angle);
    position_ticks_pub.publish(&position_ticks);
    output_pwm_pub.publish(&output_pwm);
  }
    
  nh.spinOnce();
  // Serial.println(frontLeft.getAngle());
  // Serial.println(frontLeft.currentPosition);
}
