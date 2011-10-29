#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/Bool.h>

ros::Publisher status_pub, goal_pub, cancel_pub;
std::vector<actionlib_msgs::GoalID> current_goals;
int ID = 0;


void cancel() {
    for (std::vector<actionlib_msgs::GoalID>::size_type i = 0; i < current_goals.size(); i++) {
        cancel_pub.publish(current_goals[i]);
    }
    current_goals.clear();
}

void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    
    move_base_msgs::MoveBaseActionGoal goal;

    actionlib_msgs::GoalID goal_id;
    std::stringstream s;
    s << "goal_" << ID;
    goal_id.id = s.str();
    goal_id.stamp = msg->header.stamp;
    
    goal.header.stamp = msg->header.stamp;
    goal.goal_id = goal_id;
    goal.goal.target_pose = *msg;
    
    goal_pub.publish(goal);
    current_goals.push_back(goal_id);
    
    ID++;
}

void cancel_cb(const std_msgs::Bool::ConstPtr& msg) {
    cancel();
}

void relais_cb(const std_msgs::Bool::ConstPtr& msg) {
    if (!msg->data) cancel();
}

void runstop_cb(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) cancel();
}

void update() {
    /* figure out how to track the status of a goal
    for (std::vector<actionlib_msgs::GoalID>::size_type i = 0; i < current_goals.size(); i++) {
        
    }
    */
    
    std_msgs::Bool status_msg;
    
    if (current_goals.size() > 0) {
        status_msg.data = true;
    }
    else {
        status_msg.data = false;
    }
    status_pub.publish(status_msg);
}

int main(int argc, char** argv){
    std::string ns = "goal_planner";
    ros::init(argc, argv, ns);
    ros::NodeHandle nh;
     
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(ns+"/goal", 10, goal_cb);
    ros::Subscriber cancel_sub = nh.subscribe<std_msgs::Bool>(ns+"/cancel", 10, cancel_cb);
    ros::Subscriber relais_sub = nh.subscribe<std_msgs::Bool>("relais", 10, relais_cb);
    ros::Subscriber runstop_sub = nh.subscribe<std_msgs::Bool>("runstop", 10, runstop_cb);
    ros::Subscriber runstop_wireless_sub = nh.subscribe<std_msgs::Bool>("runstop_wireless", 10, runstop_cb);
    
    status_pub = nh.advertise<std_msgs::Bool>(ns+"/has_goal", 10);
    goal_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 10);
    cancel_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 10);
    
    ros::Rate r(1);
    while (ros::ok()) {
        ros::spinOnce();
        update();
        r.sleep();
    }
    
    return 0;
}
