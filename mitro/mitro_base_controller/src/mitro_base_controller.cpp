#include "math.h"

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <mitro_base_controller/VelocityCommand.h>

#define PI 3.1415926536

double WHEEL_BASE;
double WHEEL_RADIUS;
double TICKS_PER_REV;

// these are acceleration in m/(s*s)
double MAX_ACC_LIN = 0.5;
double MAX_ACC_ROT = 1.5;

ros::Publisher cmd_vel_pub;
double vel_rot;
double vel_linear;

double last_vel_rot;
double last_vel_linear;

ros::Time last_time;
bool receiving = false;

void cmd_twist_cb(const geometry_msgs::Twist::ConstPtr& msg) {
    vel_rot = msg->angular.z;
    vel_linear = msg->linear.x;
    last_time = ros::Time::now();
    receiving = true;
}

void update() {
  mitro_base_controller::VelocityCommand vc;
  double dt = (ros::Time::now() - last_time).toSec();
  if ( dt < 0.5 ) {

    double limit_vel_rot = last_vel_rot + std::min(std::max((vel_rot - last_vel_rot), -MAX_ACC_ROT * dt), MAX_ACC_ROT * dt); 
    double limit_vel_linear = last_vel_linear + std::min(std::max((vel_linear - last_vel_linear), -MAX_ACC_LIN * dt), MAX_ACC_LIN * dt);
//    double limit_vel_rot = vel_rot;
//    double limit_vel_linear = vel_linear;

    vc.velocity_left = limit_vel_linear / WHEEL_RADIUS - 0.5 * limit_vel_rot * WHEEL_BASE / WHEEL_RADIUS; 
    vc.velocity_right = limit_vel_linear / WHEEL_RADIUS  + 0.5 * limit_vel_rot * WHEEL_BASE / WHEEL_RADIUS;
    vc.velocity_right *= 1 / (2 * PI) * TICKS_PER_REV / 2.0;
    vc.velocity_left *= 1 / (2 * PI) * TICKS_PER_REV / 2.0;
    last_vel_rot = limit_vel_rot;
    last_vel_linear = limit_vel_linear;

  }
  else {
    if (receiving) {
      ROS_WARN("Did not receive Twist message in time!");
      receiving = false;
    }
    last_vel_rot = vel_rot = 0;
    last_vel_linear = vel_linear = 0;
    vc.velocity_right = 0;
    vc.velocity_left = 0;
  }
  cmd_vel_pub.publish(vc);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mitro_base_controller");
  ros::NodeHandle n;

  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_twist", 10, cmd_twist_cb);
  cmd_vel_pub = n.advertise<mitro_base_controller::VelocityCommand>("cmd_vel", 10);
  
  last_vel_rot = vel_rot = 0;
  last_vel_linear = vel_linear = 0;
    
  try {
    n.getParam("wheel_base", WHEEL_BASE);
    n.getParam("wheel_radius", WHEEL_RADIUS);
    n.getParam("ticks_per_revolution", TICKS_PER_REV);
  }
  catch (ros::InvalidNameException e) {
    ROS_ERROR("Parameter not set: %s", e.what());
    return 1;
  }

  last_time = ros::Time::now();
    
  ros::Rate r(100);
  while (ros::ok()) {
    update();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
