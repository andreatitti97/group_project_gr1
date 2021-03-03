#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdio.h>


using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
 
  ros::Publisher vel = n.advertise<geometry_msgs::Twist>("/Box/cmd_vel", 1);
  tf::TransformBroadcaster odom_broadcaster, odom_To_world;
  

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.1;
  double vy = 0.1;
  double vz = 0.0;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while (n.ok()) {

    ros::spinOnce(); // check for incoming messages
    current_time = ros::Time::now();

    // publish the message
   
    vel.publish(msg);

    last_time = current_time;
    r.sleep();
  }
}
