#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GridCells.h>
#include <tf/transform_listener.h>

#define PI 3.141592653589793

double MAX_RANGE;
double BASE_RADIUS;
double CLEARING_DIST;
std::string BASE_FRAME, ODOM_FRAME;

ros::Publisher twist_pub, force_field_pub, force_obst_pub, force_twist_pub, result_twist_pub;
tf::TransformListener *tf_listener;
std::vector<float> force_vector(2);
ros::Time last_fv;

std::vector<float> point_to_vector(float x, float y);
double modulus(double a, double b);


void twist_cb(const geometry_msgs::Twist::ConstPtr& msg) {
    if (msg->linear.x != 0) {
        ros::Time stamp = ros::Time::now();
        
        geometry_msgs::Twist twist_msg;
        float twist_x = cos(msg->angular.z);
        float twist_y = sin(msg->angular.z);
        float twist_safe_x = twist_x + force_vector[0];
        float twist_safe_y = twist_y + force_vector[1];
        
        // publish twist and twist_safe as Pose
        float twist_yaw = atan(twist_y / twist_x);
        if (twist_x < 0) twist_yaw = modulus(twist_yaw + PI, 2*PI);
        float twist_safe_yaw = atan(twist_safe_y / twist_safe_x);
        if (twist_safe_x < 0) twist_safe_yaw = modulus(twist_safe_yaw + PI, 2*PI);
        
        geometry_msgs::PoseStamped twist_pose, twist_safe_pose, twist_pose_trans, twist_safe_pose_trans;
        twist_pose.header.stamp = twist_safe_pose.header.stamp = stamp;
        twist_pose.header.frame_id = twist_safe_pose.header.frame_id = BASE_FRAME;
        twist_pose.pose.orientation = tf::createQuaternionMsgFromYaw(twist_yaw);
        twist_safe_pose.pose.orientation = tf::createQuaternionMsgFromYaw(twist_safe_yaw);
        tf_listener->waitForTransform(BASE_FRAME, ODOM_FRAME, stamp, ros::Duration(0.1));
        try {
            tf_listener->transformPose(ODOM_FRAME, twist_pose, twist_pose_trans);
            tf_listener->transformPose(ODOM_FRAME, twist_safe_pose, twist_safe_pose_trans);
        }
        catch (tf::ExtrapolationException e) {
            ROS_ERROR("Unable to get tf transform: %s", e.what());
            return;
        }
        force_twist_pub.publish(twist_pose_trans);
        result_twist_pub.publish(twist_safe_pose_trans);
    }
    else {
        //twist_pub.publish(*msg);
    }
}

void costmap_cb(const nav_msgs::GridCells::ConstPtr& msg) {
    ODOM_FRAME = msg->header.frame_id;
    
    geometry_msgs::PoseArray ff_msg;
    ff_msg.header.stamp = msg->header.stamp;
    ff_msg.header.frame_id = ODOM_FRAME;
    
    last_fv = msg->header.stamp;
    force_vector[0] = force_vector[1] = 0;
    std::vector<geometry_msgs::Point> points = msg->cells;
    
    std::vector<geometry_msgs::Pose> poses;
    
    int count = 0;
    float length = 0.0;
    
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
        
        std::vector<float> point_vector = point_to_vector(x, y);
        
        if (point_vector.size() > 0) {
            count++;
            float weight = 0.75*(MAX_RANGE - BASE_RADIUS - CLEARING_DIST)/(sqrt(pow(x, 2) + pow(y, 2)) - BASE_RADIUS - CLEARING_DIST);
            if ( weight < 0 )             
                std::cout << "evil" << std::endl;
            length += weight;
        
            force_vector[0] += weight*point_vector[0];
            force_vector[1] += weight*point_vector[1];
            
            float yaw = atan(y / x);
            if (x > 0) yaw = modulus(yaw + PI, 2*PI);
            
            geometry_msgs::PoseStamped pose, pose_trans;
            pose.header.stamp = msg->header.stamp;
            pose.header.frame_id = BASE_FRAME;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

            tf_listener->waitForTransform(ODOM_FRAME, BASE_FRAME, pose.header.stamp, ros::Duration(0.1));
            try {
                tf_listener->transformPose(ODOM_FRAME, pose, pose_trans);
            }
            catch (tf::ExtrapolationException e) {
                ROS_ERROR("Unable to get tf transform: %s", e.what());
                return;
            }
            
            poses.push_back(pose_trans.pose);
        }
    }
    
    //force_vector[0] /= length + 1;
    //force_vector[1] /= length + 1;
    
    std::cout << force_vector[0] << "," << force_vector[1] << "," << length/(length+1) << std::endl;
    
    // publish resulting force vector as Pose
    geometry_msgs::PoseStamped force, force_trans;
    force.header.stamp = msg->header.stamp;
    force.header.frame_id = BASE_FRAME;
    float force_yaw = atan(force_vector[1] / force_vector[0]);
    if (force_vector[0] < 0) force_yaw = modulus(force_yaw + PI, 2*PI);
    force.pose.orientation = tf::createQuaternionMsgFromYaw(force_yaw);    
    tf_listener->waitForTransform(BASE_FRAME, ODOM_FRAME, msg->header.stamp, ros::Duration(0.1));
    try {
        tf_listener->transformPose(ODOM_FRAME, force, force_trans);
    }
    catch (tf::ExtrapolationException e) {
        ROS_ERROR("Unable to get tf transform: %s", e.what());
        return;
    }
    force_obst_pub.publish(force_trans);    
    
    // publish force field as PoseArray
    ff_msg.poses = poses;  
    force_field_pub.publish(ff_msg);
}

std::vector<float> point_to_vector(float x, float y) {
    std::vector<float> point_vector;
    float dist = sqrt(pow(x, 2) + pow(y, 2));
    if (dist <= MAX_RANGE) {
        point_vector.resize(2);
        point_vector[0] = -x;
        point_vector[1] = -y;
    }
    return point_vector;
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
        priv_nh.getParam("max_range", MAX_RANGE);
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
    twist_pub = nh.advertise<geometry_msgs::Twist>(target_topic, 10);
    force_field_pub = nh.advertise<geometry_msgs::PoseArray>("force_field", 10);
    force_obst_pub = nh.advertise<geometry_msgs::PoseStamped>("force_obst", 10);
    force_twist_pub = nh.advertise<geometry_msgs::PoseStamped>("force_twist", 10);
    result_twist_pub = nh.advertise<geometry_msgs::PoseStamped>("result_twist", 10);

    ros::spin();
    return 0;
}
