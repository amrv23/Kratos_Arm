#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#define SERIAL_BAUDRATE 57600

int pwm_act1_pin = 0;
int dir_act1_pin = 4;
int pwm_act2_pin = 32;
int dir_act2_pin = 33;
int pwm_base_pin = 15;
int dir_base_pin = 5;
int pwm_gripper_pin = 13;
int dir_gripper_pin = 27;
int pwm_bevel1_pin = 25;
int dir_bevel1_pin = 26;
int pwm_bevel2_pin = 14;
int dir_bevel2_pin = 12;

ros::NodeHandle nh;

// Callback function 
void messageCb(const std_msgs:Int32MultiArray &msg)  
  int act1 = msg.data[1];
  int act2 = msg.data[0];
  int bevel1 = msg.data[3];
  int bevel2 = msg.data[4];
  int base1 = msg.data[2];
  int base2 = msg.data[5];
  int grip1 = map(msg.data[6],-1,1,0,255;
  int grip2 = msg.data[7];

    // Actuator 1 
  if (act1 == 0) {
    analogWrite(pwm_act1_pin, 0);
  }
  else {
    if(act1 > 0){
      digitalWrite(dir_act1_pin, HIGH);
    }
    else{
      digitalWrite(dir_act1_pin,LOW);
    }
    analogWrite(pwm_act1pin,act1);
  }

    // Actuator 2
  if (act2 == 0) {
    analogWrite(pwm_act2_pin, 0);
  }
  else {
    if(act2 > 0){
      digitalWrite(dir_act2_pin, HIGH);
    }
    else{
      digitalWrite(dir_act2_pin,LOW);
    }
    analogWrite(pwm_act2_pin, act2);
  }

    // bevels
  if (bevel1 == 0) {
    analogWrite(pwm_bevel1_pin, 0);
  }
  if (bevel2 == 0){
    analogWrite(pwm_bevel2_pin, 0);
  }
  if(abs(bevel1) > abs(bevel2)){
    if(bevel1 > 0){
      digitalWrite(dir_bevel1_pin, HIGH);
      digitalWrite(dir_bevel2_pin, HIGH);
      analogWrite(pwm_bevel1_pin, bevel1);
    }
    else{
      digitalWrite(dir_bevel1_pin, LOW);
      digitalWrite(dir_bevel2_pin, LOW);
      analogWrite(pwm_bevel1_pin, (-1*bevel1));
    }
  }
  if(abs(bevel1) < abs(bevel2)){
    if(bevel2 > 0){
      digitalWrite(dir_bevel1_pin, HIGH);
      digitalWrite(dir_bevel2_pin, LOW);
      analogWrite(pwm_bevel1_pin, bevel2);
    }
    else{
      digitalWrite(dir_bevel1_pin, LOW);
      digitalWrite(dir_bevel2_pin, HIGH);
      analogWrite(pwm_bevel1_pin, (-1*bevel2));
    }
  }
    // Gripper control
  if (grip1 == 0 && grip2 == 0) {
    analogWrite(pwm_gripperpin, 0);
  }
  else {
    if(gripper1 == 1){
      digitalWrite(dir_basepin, HIGH);
      analogWrite(pwm_basepin, 255);
    }
    else if(gripper2 == 1){
      digitalWrite(dir_basepin, LOW);
      analogWrite(pwm_basepin, 255);
    }
  }

    // Base control
  if (base1 == 0 && base2 == 0) {
    analogWrite(pwm_basepin, 0);
  }
  else {
    if(base1 > 0){
      digitalWrite(dir_basepin, HIGH);
      analogWrite(pwm_basepin, base1);
    }
    else if(base2 > 0){
      digitalWrite(dir_basepin, LOW);
      analogWrite(pwm_basepin, base2);
    }
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("analog", messageCb);

void setup() {
  nh.getHardware()->setBaud(SERIAL_BAUDRATE);
  nh.initNode();
  nh.subscribe(sub);

  pinMode(pwm_act1_pin, OUTPUT);
  pinMode(dir_act1_pin, OUTPUT);
  pinMode(pwm_act2_pin, OUTPUT);
  pinMode(dir_act2_pin, OUTPUT);

  pinMode(pwm_base_in, OUTPUT);
  pinMode(dir_base_pin, OUTPUT);

  pinMode(pwm_gripper_pin, OUTPUT);
  pinMode(dir_gripper_pin, OUTPUT);

  pinMode(pwm_bevel1_pin, OUTPUT);
  pinMode(dir_bevel1_pin, OUTPUT);
  pinMode(pwm_bevel2_pin, OUTPUT);
  pinMode(dir_bevel2_pin, OUTPUT);

  analogWrite(pwm_act1_pin, 0);
  analogWrite(pwm_act2_pin, 0);
  analogWrite(pwm_base_pin, 0);
  analogWrite(pwm_gripper_pin, 0);
  analogWrite(pwm_bevel1_pin, 0);
  analogWrite(pwm_bevel2_pin, 0);
}

void loop(){
  nh.spinOnce();  
  delay(10);     
}