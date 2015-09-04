#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>

#define wheel_diameter  0.065 //m
#define wheel_width     27
#define axis_length     0.21  //m
#define max_RPM         298
#define pi              3.14159265

int rpm_act1 = 0;
int rpm_act2 = 0;
float x_pos = 0.0;
float y_pos = 0.0;
float theta = 0.0;
ros::Time current_time, last_time;
double dt = 0.0;

void handle_rpm( const geometry_msgs::Vector3Stamped& rpm) {
  rpm_act1 = int(rpm.vector.x);
  rpm_act2 = int(rpm.vector.y);
  dt = double(rpm.vector.z);
  current_time = rpm.header.stamp;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_driver");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("rpm", 50, handle_rpm);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster broadcaster;
  
  float rate = 10.0;
  double dx = 0.0;
  double dy = 0.0;
  double dth = 0.0;
  double dxy_ave = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  char base_link[] = "/base_link";
  char odom[] = "/odom";
  current_time = ros::Time::now();

  if (argc > 1)  sscanf(argv[1], "%f", &rate); 
  ros::Rate r(rate);

  while(n.ok()){
    // if(dt > 0.0) {
      ros::spinOnce();               // check for incoming messages
      //compute odometry in a typical way given the velocities of the robot
      dxy_ave = (rpm_act1+rpm_act2)*dt*wheel_diameter*pi/(60*2);
      dth = (rpm_act1-rpm_act2)*dt*wheel_diameter*pi/(60*axis_length);
      dx = cos(dth) * dxy_ave;
      dy = -sin(dth) * dxy_ave;
      x_pos += (cos(theta) * dx - sin(theta) * dy);
      y_pos += (sin(theta) * dx + cos(theta) * dy);
      theta += dth;

    geometry_msgs::TransformStamped t;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
    t.transform.translation.x = x_pos;
    t.transform.translation.y = y_pos;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom_quat;
    t.header.stamp = current_time;

    broadcaster.sendTransform(t);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = dxy_ave/dt;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = dth/dt;

    odom_pub.publish(odom_msg);
    dt = 0.0;
    r.sleep();
    //  }
  }
}
