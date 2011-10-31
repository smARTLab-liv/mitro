#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/GridCells.h>
#include <tf/transform_listener.h>

#define PI 3.141592653589793

double BASE_RADIUS;
double CLEARING_DIST;
std::string BASE_FRAME, ODOM_FRAME;

ros::Publisher twist_pub;
tf::TransformListener *tf_listener;
bool obst_detected = false;
ros::Time last_costmap;

bool use_assisted_drive = true;

double modulus(double a, double b);


void cmd_ad_cb(const std_msgs::Bool::ConstPtr& msg) {
    use_assisted_drive = msg->data;
    if (use_assisted_drive) ROS_DEBUG("Assisted drive mode enabled");
    else ROS_DEBUG("Assisted drive mode disabled");
}

void twist_cb(const geometry_msgs::Twist::ConstPtr& msg) {
    if (use_assisted_drive) {
        geometry_msgs::Twist twist_msg;
        if ( (ros::Time::now() - last_costmap).toSec() > 1.0) {
            ROS_ERROR("Haven't seen any costmap in the last second!");
            twist_msg.linear.x = 0;
            twist_msg.angular.z = msg->angular.z;
        }
        else if (obst_detected and msg->linear.x > 0.0) {
            ROS_ERROR("Obstacle detected!");
            twist_msg.linear.x = 0;
            twist_msg.angular.z = msg->angular.z;
        }
        else {
            twist_msg.linear.x = msg->linear.x;
            twist_msg.angular.z = msg->angular.z;
        }
        twist_pub.publish(twist_msg);
    }
    else {
        twist_pub.publish(*msg);
    }
}

void costmap_cb(const nav_msgs::GridCells::ConstPtr& msg) {
    if (use_assisted_drive) {
        ODOM_FRAME = msg->header.frame_id;
        
        last_costmap = msg->header.stamp;
        std::vector<geometry_msgs::Point> points = msg->cells;
        
        for (std::vector<std::string>::size_type i = 0; i < points.size(); i++) {
            geometry_msgs::PointStamped point, point_trans;
            point.header.stamp = msg->header.stamp;
            point.header.frame_id = ODOM_FRAME;
            point.point.x = points[i].x;
            point.point.y = points[i].y;
            tf_listener->waitForTransform(BASE_FRAME, ODOM_FRAME, point.header.stamp, ros::Duration(0.1));
            try {
                tf_listener->transformPoint(BASE_FRAME, point, point_trans);
            }
            catch (tf::ExtrapolationException e) {
                ROS_ERROR("Unable to get tf transform: %s", e.what());
                return;
            }
            
            float x = point_trans.point.x;
            float y = point_trans.point.y;
            float dist = sqrt(pow(x, 2) + pow(y, 2));
            //float yaw = atan(y / x);
            //if (x > 0) yaw = modulus(yaw + PI, 2*PI);
            
            if (dist < BASE_RADIUS + CLEARING_DIST and x > 0) {
                obst_detected = true;
            }
            else {
                obst_detected = false;
            }
        }
    }
    else obst_detected = false;
}

double modulus(double a, double b) {
    int result = static_cast<int>( a / b );
    return a - static_cast<double>( result ) * b;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "assisted_drive");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    
    std::string source_topic, target_topic, costmap_topic;
    
    try {
        priv_nh.getParam("source_topic", source_topic);   
        priv_nh.getParam("target_topic", target_topic);
        priv_nh.getParam("costmap_topic", costmap_topic);
        priv_nh.getParam("clearing_dist", CLEARING_DIST);
        priv_nh.getParam("base_frame_id", BASE_FRAME);
        double diameter;
        nh.getParam("base_diameter", diameter);
        BASE_RADIUS = diameter / 2;
    }
    catch (ros::Exception e) {
        ROS_ERROR("Parameter not set: %s", e.what());
        return 1;
    }
    
    tf_listener = new tf::TransformListener(nh);
    
    ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::Twist>(source_topic, 10, twist_cb);
    ros::Subscriber costmap_sub = nh.subscribe<nav_msgs::GridCells>(costmap_topic, 10, costmap_cb);
    ros::Subscriber cmd_ad_sub = nh.subscribe<std_msgs::Bool>("cmd_assisted_drive", 10, cmd_ad_cb);
    twist_pub = nh.advertise<geometry_msgs::Twist>(target_topic, 10);

    ros::spin();
    return 0;
}
