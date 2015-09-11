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
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include "robot_specs.h"

//Motor Shield headers
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#define encodPinA1      3     // encoder A pin
#define encodPinB1      8     // encoder B pin
#define encodPinA2      2
#define encodPinB2      7
#define LOOPTIME        100   // PID loop time(ms)

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

unsigned long lastMilli = 0;       // loop timing 
unsigned long lastMilliPub = 0;
double rpm_req1 = 0;
double rpm_req2 = 0;
double rpm_act1 = 0;
double rpm_act2 = 0;
int direction1 = FORWARD;
int direction2 = FORWARD;
int PWM_val1 = 0;                  // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int PWM_val2 = 0;
volatile long count1 = 0;          // rev counter
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;
float Kp =   1;                    // PID proportional control Gain
float Kd =   0.4;                    // PID Derivitave control gain

ros::NodeHandle nh;

void handle_cmd( const geometry_msgs::Twist& cmd_msg) {
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;
  if (z == 0) {     // go straight
    // convert m/s to rpm
    rpm_req1 = x*60/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (x == 0) {
    // convert rad/s to rpm
    rpm_req2 = z*track_width*60/(wheel_diameter*pi*2);
    rpm_req1 = -rpm_req2;
  }
  else {
    rpm_req1 = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
  }
  if (rpm_req1 > 0) direction1 = FORWARD;
  else if (rpm_req1 < 0) direction1 = BACKWARD;
  else direction1 = RELEASE;
  
  if (rpm_req2 > 0) direction2 = FORWARD;
  else if (rpm_req2 < 0) direction2 = BACKWARD;
  else direction2 = RELEASE;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel_mux", handle_cmd);
geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);
ros::Time current_time;
ros::Time last_time;

void setup() {
 AFMS.begin();  // create with the default frequency 1.6KHz
 count1 = 0;
 count2 = 0;
 countAnt1 = 0;
 countAnt2 = 0;
 rpm_req1 = 0;
 rpm_req2 = 0;
 rpm_act1 = 0;
 rpm_act2 = 0;
 PWM_val1 = 0;
 PWM_val2 = 0;
 nh.initNode();
 nh.getHardware()->setBaud(57600);
 nh.subscribe(sub);
 nh.advertise(rpm_pub);
  
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
 motor1->setSpeed(0);
 motor2->setSpeed(0);
 motor1->run(FORWARD);
 motor1->run(RELEASE);
 motor2->run(FORWARD);
 motor2->run(RELEASE);
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
    publishRPM(time-lastMilli);
    lastMilli = time;
  }
  if(time-lastMilliPub >= LOOPTIME) {
  //  publishRPM(time-lastMilliPub);
    lastMilliPub = time;
  }
}

void getMotorData(unsigned long time)  {
 rpm_act1 = double((count1-countAnt1)*60*1000)/double(time*encoder_pulse*gear_ratio);
 rpm_act2 = double((count2-countAnt2)*60*1000)/double(time*encoder_pulse*gear_ratio);
 countAnt1 = count1;
 countAnt2 = count2;
}

int updatePid1(int command, double targetValue, double currentValue) {
double pidTerm = 0;                            // PID correction
double error=0;                                  
static double last_error1=0;                             
 error = abs(targetValue) - abs(currentValue); 
 pidTerm = (Kp * error) + (Kd * (error - last_error1));                            
 last_error1 = error;
 return constrain(command + int(pidTerm), 0, 255);
}

int updatePid2(int command, double targetValue, double currentValue) {
double pidTerm = 0;                            // PID correction
double error=0;                                  
static double last_error2=0;                             
 error = abs(targetValue) - abs(currentValue); 
 pidTerm = (Kp * error) + (Kd * (error - last_error2));                            
 last_error2 = error;
 return constrain(command + int(pidTerm), 0, 255);
}

void publishRPM(unsigned long time) {
  double signed_rpm_act1 = 0;
  double signed_rpm_act2 = 0;

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

void rencoder1()  {
  if (PINB & 0b00000001)    count1--;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      count1++;                // if (digitalRead(encodPinB1)==LOW)   count --;
}

void rencoder2()  {
  if (PIND & 0b10000000)    count2--;                // if(digitalRead(encodPinB2)==HIGH)   count ++;
  else                      count2++;                // if (digitalRead(encodPinB2)==LOW)   count --;
}
