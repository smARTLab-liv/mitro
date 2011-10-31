#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tele_msgs/JointStates.h>

#define PI 3.1415926536

double WHEEL_BASIS = 0.39;
double WHEEL_RADIUS = 0.0762;

double velocity_right = 0.0;
double velocity_left = 0.0;
int position_right = 0;
int position_left = 0;

void js_cb(tele_msgs::JointStates msg) {
  velocity_right = msg.velocity_right / 18.0 * 2.0 * PI * WHEEL_RADIUS;
  velocity_left = msg.velocity_left / 18.0 * 2.0 * PI * WHEEL_RADIUS;
  position_right = msg.position_right;
  position_left = msg.position_left;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_node");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Subscriber js_sub = n.subscribe("/joint_states", 10, js_cb);

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vth = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(60.0);
  while(n.ok()){
    current_time = ros::Time::now();

    vx = (velocity_right + velocity_left) / 2.0;
    vth = (velocity_right - velocity_left) / WHEEL_BASIS;

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = vx * cos(th) * dt;
    double delta_y = vx * sin(th) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "/odom";
    odom_trans.child_frame_id = "/base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    ros::spinOnce();
    r.sleep();
  }
}
