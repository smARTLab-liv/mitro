#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GridCells.h>
#include <tf/transform_listener.h>

double MAX_RANGE;
double BASE_RADIUS;
double CLEARING_DIST;

ros::Publisher twist_pub, force_field_pub;
tf::TransformListener *tf_listener;
std::vector<float> force_vector(2);
ros::Time last_fv;

std::vector<float> point_to_vector(float x, float y);


void twist_cb(const geometry_msgs::Twist::ConstPtr& msg) {

}

void costmap_cb(const nav_msgs::GridCells::ConstPtr& msg) {
    geometry_msgs::PoseArray ff_msg;
    ff_msg.header.stamp = msg->header.stamp;
    ff_msg.header.frame_id = msg->header.frame_id;
    
    last_fv = msg->header.stamp;
    force_vector[0] = force_vector[1] = 0;
    std::vector<geometry_msgs::Point> points = msg->cells;
    
    std::vector<geometry_msgs::Pose> poses(points.size());
    
    for (std::vector<std::string>::size_type i = 0; i < points.size(); i++) {
        float x = points[i].x;
        float y = points[i].y;
        std::vector<float> point_vector = point_to_vector(x, y);
        force_vector[0] += point_vector[0];
        force_vector[1] += point_vector[1];
        
        poses[i].position.x = x;
        poses[i].position.y = y;
        float yaw = atan(y / x);
        poses[i].orientation = tf::createQuaternionMsgFromYaw(yaw);
    }
    ff_msg.poses = poses;
    force_field_pub.publish(ff_msg);
}

std::vector<float> point_to_vector(float x, float y) {
    float dist = sqrt(pow(x, 2) + pow(y, 2));
    std::vector<float> point_vector(2);
    point_vector[0] = point_vector[1] = 0;
    if (dist <= MAX_RANGE) {
        point_vector[0] = -x / (dist + 1 - BASE_RADIUS - CLEARING_DIST);
        point_vector[1] = -y / (dist + 1 - BASE_RADIUS - CLEARING_DIST);
    }
    return point_vector;
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
        priv_nh.getParam("max_range", MAX_RANGE);
        priv_nh.getParam("clearing_dist", CLEARING_DIST);
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
    twist_pub = nh.advertise<geometry_msgs::Twist>(target_topic, 10);
    force_field_pub = nh.advertise<geometry_msgs::PoseArray>("force_field", 10);

    ros::spin();
    return 0;
}
