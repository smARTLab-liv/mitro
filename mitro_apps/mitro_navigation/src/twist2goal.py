#!/usr/bin/env python
import roslib
roslib.load_manifest("mitro_navigation")
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped

from math import cos, sin

def callback(msg):
    global pub
    pos = PoseStamped()
    pos.header.stamp = rospy.Time.now()
    pos.header.frame_id = '/base_link'
    
    
    pos.pose.position.x = msg.linear.x * cos(msg.angular.z)
    pos.pose.position.y = msg.linear.x * sin(msg.angular.z)
    q = tf.transformations.quaternion_from_euler(0.0, 0.0, msg.angular.z)
    pos.pose.orientation.x = q[0]
    pos.pose.orientation.y = q[1]
    pos.pose.orientation.z = q[2]
    pos.pose.orientation.w = q[3]
    

    
    pub.publish(pos)

if __name__ == "__main__":
    rospy.init_node('twist2goal')
    rospy.Subscriber('/cmd_twist_ps3', Twist, callback)
    global pub
    pub = rospy.Publisher('/goal_planner/goal', PoseStamped)
    rospy.spin()

