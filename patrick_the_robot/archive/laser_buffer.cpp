#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>

ros::Time current_time;
sensor_msgs::LaserScan curr_scan;
sensor_msgs::LaserScan prev_scan;

void handle_scan( const sensor_msgs::LaserScan& ls) {
  prev_scan = curr_scan;
  curr_scan = ls;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_buffer");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan_raw", 50, handle_scan);
  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
  char base_link[] = "/base_link";
  char laser_link[] = "/camera_depth_frame";
  int is_first_scan = 1;
  ros::Duration d(1.0);
  float rate = 10.0;
  if (argc > 1)  sscanf(argv[1], "%f", &rate); 
  ros::Rate r(rate);
  while(n.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();
    ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan_raw", n, d);
    if (is_first_scan == 1) {
      curr_scan.header.stamp = current_time;
      curr_scan.header.frame_id = laser_link;
      pub.publish(curr_scan);
      is_first_scan = 0;
      r.sleep();
      continue;
    }
    prev_scan.header.stamp = current_time;
    prev_scan.header.frame_id = laser_link;
    pub.publish(prev_scan);
    r.sleep();
  }
}
