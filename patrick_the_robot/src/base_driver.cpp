#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <robot_specs.h>


double rpm_act1 = 0.0;
double rpm_act2 = 0.0;
double rpm_req1 = 0.0;
double rpm_req2 = 0.0;
float x_pos = 0.0;
float y_pos = 0.0;
float theta = 0.0;
ros::Time current_time;
ros::Time rpm_time(0.0);
ros::Time last_time(0.0);

void handle_rpm( const geometry_msgs::Vector3Stamped& rpm) {
  rpm_act1 = rpm.vector.x;
  rpm_act2 = rpm.vector.y;
  //dt = rpm.vector.z;
  rpm_time = rpm.header.stamp;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_driver");

  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("rpm", 50, handle_rpm);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster broadcaster;
  
  double rate = 10.0;
  double linear_scale = 1.0;
  double angular_scale = 1.0;
  bool publish_tf = true;
  double dt = 0.0;
  double dx = 0.0;
  double dy = 0.0;
  double dth = 0.0;
  double dxy_ave = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  char base_link[] = "/base_link";
  char odom[] = "/odom";
  ros::Duration d(1.0);
  nh_private_.getParam("publish_rate", rate);
  nh_private_.getParam("publish_tf", publish_tf);
  nh_private_.getParam("linear_scale", linear_scale);
  nh_private_.getParam("angular_scale", angular_scale);

  ros::Rate r(rate);
  while(n.ok()){
    ros::spinOnce();
    ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>("rpm", n, d);
    current_time = ros::Time::now();
    dt = (current_time-last_time).toSec();
    dxy_ave = (rpm_act1+rpm_act2)*dt*wheel_diameter*pi/(60*2);
    dth = (rpm_act2-rpm_act1)*dt*wheel_diameter*pi/(60*track_width);
    if (dth > 0) dth *= angular_scale;
    if (dxy_ave > 0) dxy_ave *= linear_scale;
    dx = cos(dth) * dxy_ave;
    dy = -sin(dth) * dxy_ave;
    x_pos += (cos(theta) * dx - sin(theta) * dy);
    y_pos += (sin(theta) * dx + cos(theta) * dy);
    theta += dth;

    if(theta >= two_pi) theta -= two_pi;
    if(theta <= -two_pi) theta += two_pi;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    if(publish_tf) {
      geometry_msgs::TransformStamped t;
      t.header.frame_id = odom;
      t.child_frame_id = base_link;
      t.transform.translation.x = x_pos;
      t.transform.translation.y = y_pos;
      t.transform.translation.z = 0.0;
      t.transform.rotation = odom_quat;
      t.header.stamp = current_time;

      broadcaster.sendTransform(t);
    }

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;
    if (rpm_act1 == 0 && rpm_act2 == 0){
      odom_msg.pose.covariance[0] = 1e-9;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 1e-9;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e-9;
      odom_msg.twist.covariance[0] = 1e-9;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 1e-9;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e-9;
    }
    else{
      odom_msg.pose.covariance[0] = 1e-3;
      odom_msg.pose.covariance[7] = 1e-3;
      odom_msg.pose.covariance[8] = 0.0;
      odom_msg.pose.covariance[14] = 1e6;
      odom_msg.pose.covariance[21] = 1e6;
      odom_msg.pose.covariance[28] = 1e6;
      odom_msg.pose.covariance[35] = 1e3;
      odom_msg.twist.covariance[0] = 1e-3;
      odom_msg.twist.covariance[7] = 1e-3;
      odom_msg.twist.covariance[8] = 0.0;
      odom_msg.twist.covariance[14] = 1e6;
      odom_msg.twist.covariance[21] = 1e6;
      odom_msg.twist.covariance[28] = 1e6;
      odom_msg.twist.covariance[35] = 1e3;
    }
    vx = (dt == 0)?  0 : dxy_ave/dt;
    vth = (dt == 0)? 0 : dth/dt;
    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = vth;

    odom_pub.publish(odom_msg);
    last_time = current_time;
    r.sleep();
  }
}
