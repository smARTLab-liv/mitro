#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "mitro_diagnostics/SysInfo.h"
#include <sstream>
#include <stdio.h>
#include <string>
#include <iostream>

ros::Publisher twist_pub;

float FORWARD_SPEED = 0.3;
float BACKWARD_SPEED = 0.1;
float CURVE_TURNING_SPEED = 0.5;
float INPLACE_TURNING_SPEED = 0.5;

void sysinfo_cb(const mitro_diagnostics::SysInfo::ConstPtr& msg) {
    float battery_voltage = msg->battery_voltage;
    float signal_level = msg->wifi_signallevel;
    float signal_percent = 100 - (signal_level - 30) * 100.0 / 65.0;
    printf("BAT%f:", std::max(battery_voltage, 15.0f) / 15.0);
    printf("SIG%f:", signal_percent / 100.0);
    fflush(stdout);
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    printf("SPD%f:",msg->twist.twist.linear.x);
    fflush(stdout);
}

void update(const ros::TimerEvent& event){

	char string [2]; //Buffer for message
	geometry_msgs::Twist msg; //Twist message to be published
	int x; //Number of characters read

	//While ros is ok
	while (ros::ok()){

		//Read command from stdin	
		x=scanf("%s",string);

		//If command is 2 characters long as expected
		if (x>0){
			
			//Translate command to twist message and send
			if (!strcmp(string, "FW")){
				//printf("FW"); 
				msg.linear.x=FORWARD_SPEED;
				msg.angular.z=0.0f;
				twist_pub.publish(msg);
			}
	        else if (!strcmp(string, "FL")){
	      		//printf("FL");
                msg.linear.x=FORWARD_SPEED;
                msg.angular.z=CURVE_TURNING_SPEED;
                twist_pub.publish(msg);
	        }
	        else if (!strcmp(string, "FR")){
                //printf("FR");
                msg.linear.x=FORWARD_SPEED;
                msg.angular.z=-CURVE_TURNING_SPEED;
                twist_pub.publish(msg);
	        }
	        else if (!strcmp(string, "LE")){
                //printf("LE");
                msg.linear.x=0.0f;
                msg.angular.z=INPLACE_TURNING_SPEED;
                twist_pub.publish(msg);
	        }
	        else if (!strcmp(string, "RI")){
                //printf("RI");
                msg.linear.x=0.0f;
                msg.angular.z=-INPLACE_TURNING_SPEED;
                twist_pub.publish(msg);
	        }
	        else if (!strcmp(string, "BK")){
                //printf("BK");
                msg.linear.x=-BACKWARD_SPEED;
                msg.angular.z=0.0f;
                twist_pub.publish(msg);
	        }
	        else if (!strcmp(string, "ST")){
                //printf("ST");
                msg.linear.x=0.0f;
                msg.angular.z=0.0f;
                twist_pub.publish(msg);
	        }
		}
	}
}


int main(int argc, char **argv){

	//Ros initialization
	ros::init(argc, argv, "tele_wrapper");
	
	//get Node handle
	ros::NodeHandle n;

	//Publish motor messages 
	twist_pub = n.advertise<geometry_msgs::Twist>("cmd_twist_tele", 10);
    
    //Subscribe to SysInfo
    ros::Subscriber sysinfo = n.subscribe("sysinfo", 10, sysinfo_cb);
    
    //Subscribe to Odom
    ros::Subscriber odom = n.subscribe("odom", 10, odom_cb);
	
	//Start motor command listener thread
	ros::Timer t = n.createTimer(ros::Duration(0.1), update);

	// Use 3 threads
	ros::MultiThreadedSpinner spinner(3);

	//Spin the shit out of it
	spinner.spin(); 

	//Quit if node was shut down or CTRL-C was pressed
	return 0;
}
