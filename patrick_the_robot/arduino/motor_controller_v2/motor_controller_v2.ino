// Test MD03a / Pololu motor with encoder
// speed control (PI), V & I display
// Credits:
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923

//ROS headers
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

//Motor Shield headers
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#define encodPinA1      3     // encoder A pin
#define encodPinB1      8     // encoder B pin
#define encodPinA2      2
#define encodPinB2      7
#define LOOPTIME        100   // PID loop time(ms)
#define encoder_pulse   13
#define gear_ratio      20
#define wheel_diameter  0.069 //m
#define wheel_width     27
#define axis_length     0.276  //m
#define max_RPM         298
#define pi              3.14159265

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

unsigned long lastMilli = 0;       // loop timing 
unsigned long lastMilliPrint = 0;
unsigned long lastMilliPub = 0;
int rpm_req1 = 0;
int rpm_req2 = 0;
int rpm_act1 = 0;
int rpm_act2 = 0;
int direction1 = FORWARD;
int direction2 = FORWARD;
int PWM_val1 = 0;                  // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2 = 0;
volatile long count1 = 0;          // rev counter
volatile long count2 = 0;
float Kp =   1;                    // PID proportional control Gain
float Kd =   0.5;                    // PID Derivitave control gain
float x_pos = 0.0;
float y_pos = 0.0;
float theta = 0.0;
char base_link[] = "/base_link";
char odom[] = "/odom";

ros::NodeHandle nh;

void handle_cmd( const geometry_msgs::Twist& cmd_msg) {
  float x = cmd_msg.linear.x;
  float z = cmd_msg.angular.z;
  if (z == 0) {     // go straight
    // convert m/s to rpm
    rpm_req1 = x*60/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (x == 0) {
    // convert rad/s to rpm
    rpm_req2 = z*axis_length*60/(wheel_diameter*pi*2);
    rpm_req1 = -rpm_req2;
  }
  else {
    rpm_req1 = x*60/(pi*wheel_diameter)-z*axis_length*60/(wheel_diameter*pi*2);
    rpm_req2 = x*60/(pi*wheel_diameter)+z*axis_length*60/(wheel_diameter*pi*2);
  }
  if (rpm_req1 >= 0) direction1 = FORWARD;
  else direction1 = BACKWARD;
  if (rpm_req2 >= 0) direction2 = FORWARD;
  else direction2 = BACKWARD;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd);
//geometry_msgs::TransformStamped t;
//tf::TransformBroadcaster broadcaster;
//nav_msgs::Odometry odom_msg;
geometry_msgs::Vector3Stamped rpm_msg;
//ros::Publisher odom_pub("odom", &odom_msg);
ros::Publisher rpm_pub("rpm", &rpm_msg);
ros::Time current_time;
ros::Time last_time;

void setup() {
 AFMS.begin();  // create with the default frequency 1.6KHz

 nh.initNode();
 nh.getHardware()->setBaud(57600);
 nh.subscribe(sub);
 //nh.advertise(odom_pub);
 nh.advertise(rpm_pub);
 //broadcaster.init(nh);
 
 pinMode(encodPinA1, INPUT); 
 pinMode(encodPinB1, INPUT); 
 digitalWrite(encodPinA1, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB1, HIGH);
 attachInterrupt(1, rencoder1, FALLING);

 pinMode(encodPinA2, INPUT); 
 pinMode(encodPinB2, INPUT); 
 digitalWrite(encodPinA2, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB2, HIGH);
 attachInterrupt(0, rencoder2, FALLING);
}

void loop() {
 
  nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME)   {      // enter tmed loop
    getMotorData(time-lastMilli);                             // calculate speed, volts and Amps
    PWM_val1 = updatePid1(PWM_val1, rpm_req1, rpm_act1); // compute PWM value
    PWM_val2 = updatePid2(PWM_val2, rpm_req2, rpm_act2); // compute PWM value
    motor1->setSpeed(PWM_val1);                 // send PWM to motor
    motor2->setSpeed(PWM_val2);
    motor1->run(direction1);
    motor2->run(direction2);
    lastMilli = time;
  }
  if(time-lastMilliPub >= LOOPTIME) {
   // publishOdom(time-lastMilliPub);
    publishRPM(time-lastMilliPub);
    lastMilliPub = time;
  }
 //printMotorInfo();                          // display data
}

void getMotorData(unsigned long time)  {      // calculate speed, volts and Amps
 static long countAnt1 = 0;                   // last count
 static long countAnt2 = 0;
 rpm_act1 = ((count1 - countAnt1)*(60*(1000/time)))/(encoder_pulse*gear_ratio);
 rpm_act2 = ((count2 - countAnt2)*(60*(1000/time)))/(encoder_pulse*gear_ratio);
 countAnt1 = count1;
 countAnt2 = count2;
}

int updatePid1(int command, int targetValue, int currentValue) {
float pidTerm = 0;                            // PID correction
int error=0;                                  
static int last_error1=0;                             
 error = abs(targetValue) - abs(currentValue); 
 pidTerm = (Kp * error) + (Kd * (error - last_error1));                            
 last_error1 = error;
 return constrain(command + int(pidTerm), 0, 255);
}

int updatePid2(int command, int targetValue, int currentValue) {
float pidTerm = 0;                            // PID correction
int error=0;                                  
static int last_error2=0;                             
 error = abs(targetValue) - abs(currentValue); 
 pidTerm = (Kp * error) + (Kd * (error - last_error2));                            
 last_error2 = error;
 return constrain(command + int(pidTerm), 0, 255);
}

/**
void publishOdom(unsigned long time) {
  double dx = 0.0;
  double dy = 0.0;
  double dxy_ave = 0.0;
  double dth = 0.0;
  double dt = 0.0;
  int signed_rpm_act1 = 0;
  int signed_rpm_act2 = 0;

  if(direction1 == FORWARD) signed_rpm_act1 = rpm_act1;
  else signed_rpm_act1 = -rpm_act1;

  if(direction2 == FORWARD) signed_rpm_act2 = rpm_act2;
  else signed_rpm_act2 = -rpm_act2;
  
  dt = (double)time/1000; //ms to s
  current_time = nh.now();
  dxy_ave = (signed_rpm_act1+signed_rpm_act2)*dt*wheel_diameter*pi/(60*2);
  dth = (signed_rpm_act1-signed_rpm_act2)*dt*wheel_diameter*pi/(60*axis_length);
  dx = cos(dth) * dxy_ave;
  dy = -sin(dth) * dxy_ave;
  x_pos += (cos(theta) * dx - sin(theta) * dy);
  y_pos += (sin(theta) * dx + cos(theta) * dy);
  theta += dth;
  
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);
  t.header.frame_id = base_link;
  t.child_frame_id = odom;
  t.transform.translation.x = x_pos;
  t.transform.translation.y = y_pos;
  t.transform.translation.z = 0.0;
  t.transform.rotation = odom_quat;
  t.header.stamp = current_time;

  broadcaster.sendTransform(t);
  nh.spinOnce();
  
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = odom;
  odom_msg.pose.pose.position.x = x_pos;
  odom_msg.pose.pose.position.y = y_pos;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;
  odom_msg.child_frame_id = base_link;
  odom_msg.twist.twist.linear.x = dxy_ave/dt;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.z = dth/dt;
  
  odom_pub.publish(&odom_msg);
  nh.spinOnce();
}
**/
void publishRPM(unsigned long time) {
  int signed_rpm_act1 = 0;
  int signed_rpm_act2 = 0;

  if(direction1 == FORWARD) signed_rpm_act1 = rpm_act1;
  else signed_rpm_act1 = -rpm_act1;

  if(direction2 == FORWARD) signed_rpm_act2 = rpm_act2;
  else signed_rpm_act2 = -rpm_act2;

  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = signed_rpm_act1;
  rpm_msg.vector.y = signed_rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}

void printMotorInfo()  {                                                      // display data
 if((millis()-lastMilliPrint) >= 1000)   {                     
   lastMilliPrint = millis();
   Serial.print("required RPM:");   Serial.print(rpm_req1);
   Serial.println("");
   Serial.print("  motor1 RPM:");   Serial.print(rpm_act1);
   Serial.println("");
   Serial.print("  motor1 PWM:");   Serial.print(PWM_val1);
   Serial.println("");
   Serial.print("  motor2 RPM:");   Serial.print(rpm_act2);
   Serial.println("");
   Serial.print("  motor2 PWM:");   Serial.print(PWM_val2);
   Serial.println("");
   Serial.println("-------------");
 }
}

void rencoder1()  {                                 // pulse and direction, direct port reading to save cycles
 if (PINB & 0b00000001)    count1--;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
 else                      count1++;                // if (digitalRead(encodPinB1)==LOW)   count --;                    count2--;                // if (digitalRead(encodPinB2)==LOW)   count --;
}

void rencoder2()  {                                 // pulse and direction, direct port reading to save cycles                  count1--;                // if (digitalRead(encodPinB1)==LOW)   count --;
 if (PIND & 0b10000000)    count2--;                // if(digitalRead(encodPinB2)==HIGH)   count ++;
 else                      count2++;                // if (digitalRead(encodPinB2)==LOW)   count --;
}
