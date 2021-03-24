
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <stdio.h>


using namespace std;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  double x_orient = 0.0;
  double y_orient = 0.0;
  double z_orient = 0.0;
  double w_orient = 0.0;

  bool flagOdom = false;
  
  nav_msgs::Odometry odom;

void Box_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;
    z = odom.pose.pose.position.z;

    x_orient = odom.pose.pose.orientation.x;
    y_orient = odom.pose.pose.orientation.y;
    z_orient = odom.pose.pose.orientation.z;
    w_orient = odom.pose.pose.orientation.w;
    cout << "x_orient: " << x_orient << endl;
    flagOdom = true;
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("Box/odom", 1);
  ros::Publisher vel = nh.advertise<geometry_msgs::Twist>("/mobile/cmd_vel", 1);
  ros::Subscriber odom_sub = nh.subscribe("/odom",1,Box_odom_callback);

  
  tf::TransformBroadcaster odom_broadcaster, odom_To_world;
  tf::TransformListener oLw;
  tf::StampedTransform oSw;
 
  double vx = 0.1;
  double vy = 0.1;
  double vz = 0.0;
  double vth = 0.1;
 
  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while (nh.ok()) {

    odom_To_world.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(x_orient, y_orient, z_orient, w_orient), tf::Vector3(x, y, z)),ros::Time::now(),"world","odom"));
    try
    {
     oLw.waitForTransform("world","odom",ros::Time(0), ros::Duration(1.0));
     oLw.lookupTransform("world","odom",ros::Time(0), oSw);
    } 
    catch (tf::TransformException ex) 
    {
     ROS_ERROR("%s",ex.what());
    }
     
    tf::Vector3 BoxPosition = oSw.getOrigin();
    
    x = BoxPosition.getX();
    y = BoxPosition.getY();
    z = BoxPosition.getZ();
    


   /* tf:: Transform transform;
    transform.setOrigin(tf::Vector3(odom.pose.pose.position.x ,odom.pose.pose.position.y, odom.pose.pose.position.z));
    tf:: Quaternion q;
    q.setRPY(x_orient, y_orient,z_orient);
    transform.setRotation(q);
    
    odom_To_world.sendTransform(tf::StampedTransform(transform,ros::Time::now(), "world","odom")); */
    //std::cout<<"odom_To_world---> x: "<< odom_To_world.pose.pose.position.x<<"/n";
    
    // next, we'll publish the cmd_vel message over ROS
    geometry_msgs::Twist msg;
    msg.linear.x = vx;
    msg.linear.y = vy;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
   
    // publish the message
    odom_pub.publish(odom);
    vel.publish(msg);

    last_time = current_time;
    r.sleep();
  }
}
