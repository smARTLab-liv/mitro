/*
 * mitro_joy_node
 * adapted from teleop_pr2
 */

#include <cstdlib>
#include <cstdio>
#include <unistd.h> //don't know if needed
#include <math.h> //don't know if needed
#include <fcntl.h> //don't know if needed
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

const int PUBLISH_FREQ = 20;

using namespace std;

class TeleopMITRO
{
    public:
    geometry_msgs::Twist cmd;
    double max_vx, max_vy, max_vw, max_vx_run, max_vy_run, max_vw_run;
    double req_vx, req_vy, req_vw;
    int axis_vx, axis_vy, axis_vw;
    int deadman_button, run_button;
    bool deadman_no_publish_;

    bool deadman_;
    bool last_deadman_;

    ros::Time last_recieved_joy_message_time_;
    ros::Duration joy_msg_timeout_;

    ros::NodeHandle n_, n_private_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

    TeleopMITRO(bool deadman_no_publish = false) :
        max_vx(0.5), max_vy(0.0), max_vw(1.0),
        max_vx_run(1.0), max_vy_run(0.0), max_vw_run(2.0),
        deadman_no_publish_(deadman_no_publish), 
        deadman_(false), last_deadman_(false),
        n_private_("~")
    { }

    void init()
    {
        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

        // Set max speed
        n_private_.param("max_vx", max_vx, max_vx);
        n_private_.param("max_vy", max_vy, max_vy);
        n_private_.param("max_vw", max_vw, max_vw);

        // Set max speed while running
        n_private_.param("max_vx_run", max_vx_run, max_vx_run);
        n_private_.param("max_vy_run", max_vy_run, max_vy_run);
        n_private_.param("max_vw_run", max_vw_run, max_vw_run);

        n_private_.param("axis_vx", axis_vx, 1);
        n_private_.param("axis_vw", axis_vw, 0);
        n_private_.param("axis_vy", axis_vy, 3);

        n_private_.param("deadman_button", deadman_button, 0);
        n_private_.param("run_button", run_button, 0);

        double joy_msg_timeout;
        n_private_.param("joy_msg_timeout", joy_msg_timeout, 0.5); //default to 0.5 seconds timeout
        if (joy_msg_timeout <= 0)
        {
            joy_msg_timeout_ = ros::Duration().fromSec(9999999);//DURATION_MAX;
            ROS_DEBUG("joy_msg_timeout <= 0 -> no timeout");
        }
        else
        {
            joy_msg_timeout_.fromSec(joy_msg_timeout);
            ROS_DEBUG("joy_msg_timeout: %.3f", joy_msg_timeout_.toSec());
        }

        ROS_DEBUG("max_vx: %.3f m/s\n", max_vx);
        ROS_DEBUG("max_vy: %.3f m/s\n", max_vy);
        ROS_DEBUG("max_vw: %.3f deg/s\n", max_vw*180.0/M_PI);

        ROS_DEBUG("max_vx_run: %.3f m/s\n", max_vx_run);
        ROS_DEBUG("max_vy_run: %.3f m/s\n", max_vy_run);
        ROS_DEBUG("max_vw_run: %.3f deg/s\n", max_vw_run*180.0/M_PI);

        ROS_DEBUG("axis_vx: %d\n", axis_vx);
        ROS_DEBUG("axis_vy: %d\n", axis_vy);
        ROS_DEBUG("axis_vw: %d\n", axis_vw);

        ROS_DEBUG("deadman_button: %d\n", deadman_button);
        ROS_DEBUG("run_button: %d\n", run_button);
        ROS_DEBUG("joy_msg_timeout: %f\n", joy_msg_timeout);

        vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        joy_sub_ = n_.subscribe("joy", 10, &TeleopMITRO::joy_cb, this);
    }

    ~TeleopMITRO() { }

    /** Callback for joy topic **/
    void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        //Record this message reciept
        last_recieved_joy_message_time_ = ros::Time::now();

        deadman_ = (((unsigned int)deadman_button < joy_msg->buttons.size()) && joy_msg->buttons[deadman_button]);

        if (!deadman_)
            return;

        // Base
        bool running = (((unsigned int)run_button < joy_msg->buttons.size()) && joy_msg->buttons[run_button]);
        double vx = running ? max_vx_run : max_vx;
        double vy = running ? max_vy_run : max_vy;
        double vw = running ? max_vw_run : max_vw;

        if((axis_vx >= 0) && (((unsigned int)axis_vx) < joy_msg->axes.size()))
            req_vx = joy_msg->axes[axis_vx] * vx;
        else
            req_vx = 0.0;
        if((axis_vy >= 0) && (((unsigned int)axis_vy) < joy_msg->axes.size()))
            req_vy = joy_msg->axes[axis_vy] * vy;
        else
            req_vy = 0.0;
        if((axis_vw >= 0) && (((unsigned int)axis_vw) < joy_msg->axes.size()))
            req_vw = joy_msg->axes[axis_vw] * vw;
        else
            req_vw = 0.0;

        // Enforce max/mins for velocity
        // Joystick should be [-1, 1], but it might not be
        req_vx = max(min(req_vx, vx), -vx);
        req_vy = max(min(req_vy, vy), -vy);
        req_vw = max(min(req_vw, vw), -vw);
    }

    void send_cmd_vel()
    {
        if(deadman_  &&
           last_recieved_joy_message_time_ + joy_msg_timeout_ > ros::Time::now())
        {
            // Base
            cmd.linear.x = req_vx;
            cmd.linear.y = req_vy;
            cmd.angular.z = req_vw;
            vel_pub_.publish(cmd);

            fprintf(stdout,"teleop_base:: %f, %f, %f.\n",
                cmd.linear.x, cmd.linear.y, cmd.angular.z);
        }
        else
        {
            // Publish zero commands iff deadman_no_publish is false
            cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
            if (!deadman_no_publish_)
            {
                // Base
                vel_pub_.publish(cmd);
            }
        }

        //make sure we store the state of our last deadman
        last_deadman_ = deadman_;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mitro_joy_node");
    const char* opt_no_publish    = "--deadman_no_publish";

    bool no_publish = false;
    for(int i=1;i<argc;i++)
    {
        if(!strncmp(argv[i], opt_no_publish, strlen(opt_no_publish)))
            no_publish = true;
    }

    TeleopMITRO mitro_joy_node(no_publish);
    mitro_joy_node.init();

    ros::Rate pub_rate(PUBLISH_FREQ);

    while (mitro_joy_node.n_.ok())
    {
        ros::spinOnce();
        mitro_joy_node.send_cmd_vel();
        pub_rate.sleep();
    }

    exit(0);
    return 0;
}

