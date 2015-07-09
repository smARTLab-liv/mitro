#include <ros/ros.h>
#include <mitro_sonar/SonarScan.h>
#include <mitro_sonar/SonarRaw.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>

#define PI 3.1415926536

//just checking

ros::Publisher sonar_pub;
ros::Publisher laser_pub;

std::vector<float> ranges, angles;
double min_range, max_range, arc_range, base_radius;

mitro_sonar::SonarScan sonar_msg;
sensor_msgs::LaserScan laser_msg;

void sonar_raw_cb(const mitro_sonar::SonarRaw::ConstPtr& msg) {
    sonar_msg.header.stamp = ros::Time::now();
    ranges[0] = (msg->s1_dist) / 100.0 + base_radius - 0.01;
    ranges[1] = (msg->s2_dist) / 100.0 + base_radius - 0.01;
    ranges[2] = (msg->s3_dist) / 100.0 + base_radius - 0.01;
    ranges[3] = (msg->s4_dist) / 100.0 + base_radius - 0.01;
    ranges[4] = (msg->s5_dist) / 100.0 + base_radius - 0.01;
    sonar_msg.ranges = ranges;
    sonar_pub.publish(sonar_msg);
    
    laser_msg.header.stamp = ros::Time::now();
    laser_msg.ranges = ranges;
    laser_pub.publish(laser_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mitro_sonar");
  ros::NodeHandle n;

  ros::Subscriber sonar_sub = n.subscribe("sonar_raw", 10, sonar_raw_cb);
  sonar_pub = n.advertise<mitro_sonar::SonarScan>("sonar_scan", 10);
  laser_pub = n.advertise<sensor_msgs::LaserScan>("sonar_laser", 10);
  
  ranges = std::vector<float>(5);
    
  try {
    double s1_angle, s2_angle, s3_angle, s4_angle, s5_angle;
    n.getParam("mitro_sonar/s1_angle", s1_angle);
    n.getParam("mitro_sonar/s2_angle", s2_angle);
    n.getParam("mitro_sonar/s3_angle", s3_angle);
    n.getParam("mitro_sonar/s4_angle", s4_angle);
    n.getParam("mitro_sonar/s5_angle", s5_angle);
    n.getParam("mitro_sonar/sonar_min", min_range);
    n.getParam("mitro_sonar/sonar_max", max_range);
    n.getParam("mitro_sonar/sonar_arc", arc_range);
    n.getParam("base_diameter", base_radius);
    base_radius /= 2.0;
    
    angles = std::vector<float>(5);
    angles[0] = s1_angle;
    angles[1] = s2_angle;
    angles[2] = s3_angle;
    angles[3] = s4_angle;
    angles[4] = s5_angle;
    
    sonar_msg.header.frame_id = "base_sonar";
    sonar_msg.arc_range = arc_range;
    sonar_msg.min_range = min_range;
    sonar_msg.max_range = max_range;
    sonar_msg.angles = angles;
    
    laser_msg.header.frame_id = "base_sonar";
    laser_msg.angle_min = s1_angle;
    laser_msg.angle_max = s1_angle + 2.0;
    laser_msg.angle_increment = 0.5;
    laser_msg.range_min = min_range;
    laser_msg.range_max = max_range;
    laser_msg.scan_time = 0.12; // estimated guess
    laser_msg.time_increment = 0.12 / 5.0; // estimated guess
    
  }
  catch (ros::InvalidNameException e) {
    ROS_ERROR("Parameter not set: %s", e.what());
    return 1;
  }

  ros::Rate r(100);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
