#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <mitro_base_controller/JointStates.h>

#define PI 3.1415926536

double WHEEL_BASE;
double WHEEL_RADIUS;
double TICKS_TO_DIST;
std::string FRAME_ID;
std::string CHILD_FRAME_ID;

class Odom {
    //double velocity_right;
    //double velocity_left;
    int position_right;
    int position_left;
    int last_position_right;
    int last_position_left;
    //double cmd_angular;
    //double cmd_linear;

    double x, y, th, vx, vth;

    ros::Time last_stamp, current_time, last_time;
    tf::TransformBroadcaster odom_broadcaster;

public:
    Odom();
    void js_cb(const mitro_base_controller::JointStates::ConstPtr& msg);
    //void cmd_twist_cb(const geometry_msgs::Twist::ConstPtr& msg);
    void update();

    ros::Publisher odom_pub;
};

Odom::Odom () {
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    //velocity_right = velocity_left = 0.0d;
    position_right = position_left = 0;
    last_position_right = last_position_left = 0;
    x = y = th = vx = vth = 0;
    //cmd_angular = cmd_linear = 0;
}

void Odom::js_cb(const mitro_base_controller::JointStates::ConstPtr& msg) {
    //velocity_right = msg->velocity_right * 2.0 * TICKS_TO_DIST;
    //velocity_left = msg->velocity_left * 2.0 * TICKS_TO_DIST;
    position_right = msg->position_right;
    position_left = msg->position_left;

    ros::Time current_time = msg->header.stamp;

    double dt = (current_time - last_time).toSec();

    // as long as we don't drive 300km straight, this should be fine.
    double vl = (double)(position_left - last_position_left) * TICKS_TO_DIST;
    double vr = (double)(position_right - last_position_right) * TICKS_TO_DIST;

    vx = (vr + vl) / 2.0;
    vth = (vr - vl) / WHEEL_BASE;

    double delta_x = vx * cos(th);
    double delta_y = vx * sin(th);
    double delta_th = vth;

    x += delta_x;
    y += delta_y;

    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = FRAME_ID;
    odom_trans.child_frame_id = CHILD_FRAME_ID;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();// this was current_time before, but time with micro does not seem to be in sync
    odom.header.frame_id = FRAME_ID;

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //get filtered velocity for twist
    //vx = (velocity_right + velocity_left) / 2.0;
    //vth = (velocity_right - velocity_left) / WHEEL_BASE;
    vx = vx / dt;
    vth = vth / dt;

    odom.child_frame_id = CHILD_FRAME_ID;
    odom.twist.twist.linear.x = vx;//cmd_linear;//(vx + cmd_linear) / 2.0;
    odom.twist.twist.angular.z = vth;//cmd_angular;//(vth + cmd_angular) / 2.0;

    //publish the message
    odom_pub.publish(odom);

    last_position_left = position_left;
    last_position_right = position_right;

    last_time = current_time;
}

//void Odom::cmd_twist_cb(const geometry_msgs::Twist::ConstPtr& msg) {
//    cmd_angular = msg->angular.z;
//    cmd_linear = msg->linear.x;
//}

int main(int argc, char** argv){
    ros::init(argc, argv, "mitro_odom");
    ros::NodeHandle n;
    Odom odom = Odom();

    ros::Subscriber js_sub = n.subscribe("joint_states", 10, &Odom::js_cb, &odom);
    //ros::Subscriber cmd_twist_sub = n.subscribe("cmd_twist_mixed", 10, &Odom::cmd_twist_cb, &odom);

    odom.odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);;

    int ticksPerRev;
    try {
        n.getParam("wheel_base", WHEEL_BASE);
        n.getParam("wheel_radius", WHEEL_RADIUS);
        n.getParam("ticks_per_revolution", ticksPerRev);
        ros::NodeHandle np("~");
        np.getParam("frame_id", FRAME_ID);
        np.getParam("child_frame_id", CHILD_FRAME_ID);
        TICKS_TO_DIST = PI * WHEEL_RADIUS / ((double)ticksPerRev / 2.0);
    }
    catch (ros::InvalidNameException e) {
        ROS_ERROR("Parameter not set: %s", e.what());
        return 1;
    }

    ros::spin();
    return 0;
}
