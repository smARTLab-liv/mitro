// should listen to some PoseStamped goal from RVIZ, publish it as action goal, and be able to cancel this goal.

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/Bool.h>

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher status_pub, goal_pub, cancel_pub;
//actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> goal_client("move_base", true);
//actionlib::SimpleActionClient<actionlib_msgs::GoalID> cancel_client("move_base", true);
std::vector<actionlib_msgs::GoalID> current_goals;


//void done_cb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseActionResultConstPtr& result);
//void active_cb();
//void feedback_cb(const move_base_msgs::MoveBaseActionFeedbackConstPtr& feedback);


void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    /*
    MoveBaseClient ac("move_base", true);
    
    ROS_INFO("Waiting for move_base action server.");
    ac.waitForServer();
    ROS_INFO("Action server started.");
    */
    
    move_base_msgs::MoveBaseActionGoal goal;

    actionlib_msgs::GoalID goal_id;
    std::stringstream s;
    s << "goal_" << current_goals.size();
    goal_id.id = s.str();
    goal_id.stamp = msg->header.stamp;
    
    goal.header.stamp = msg->header.stamp;
    goal.goal_id = goal_id;
    goal.goal.target_pose = *msg;
    
    //ac.sendGoal(goal, &done_cb, &active_cb, &feedback_cb);
    goal_pub.publish(goal);
    current_goals.push_back(goal_id);
}

void cancel_cb(const std_msgs::Bool::ConstPtr& msg) {
    for (std::vector<actionlib_msgs::GoalID>::size_type i = 0; i < current_goals.size(); i++) {
        cancel_pub.publish(current_goals[i]);
    }
    current_goals.clear();
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

/*
// Called once when the goal completes
void done_cb(const actionlib::SimpleClientGoalState& state, const MoveBaseActionResultConstPtr& result) {
    ROS_INFO("Move Base finished goal %d",result->goal_id.id);
}

// Called once when the goal becomes active
void active_cb() {
    ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedback_cb(const move_base_msgs::MoveBaseActionFeedbackConstPtr& feedback) {
  ROS_INFO("Got Feedback");
}
*/

int main(int argc, char** argv){
    std::string ns = "goal_planner";
    ros::init(argc, argv, ns);
    ros::NodeHandle nh;
     
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(ns+"/goal", 10, goal_cb);
    ros::Subscriber cancel_sub = nh.subscribe<std_msgs::Bool>(ns+"/cancel_goal", 10, cancel_cb);
    ros::Subscriber relais_sub = nh.subscribe<std_msgs::Bool>("relais", 10, cancel_cb);
    ros::Subscriber runstop_sub = nh.subscribe<std_msgs::Bool>("runstop", 10, cancel_cb);
    
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
