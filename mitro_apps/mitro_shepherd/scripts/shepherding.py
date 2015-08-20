#!/usr/bin/env python
from socket import gethostname

import rospy
import tf

from geometry_msgs.msg import PoseStamped
from swarming_turtles_msgs.msg import Turtles, CommunicationProtocol
from mitro_shepherd.srv import *
from std_msgs.msg import Bool

turtles = []
own_name = gethostname()  # hostname used for communication

tf_listener = None
comm_pub = None

HIVE_FRAME = rospy.get_param('hive_frame', '/hive')
food = None

active = False


def transform_pose(pose_in, frame=HIVE_FRAME):
    if tf_listener.frameExists(pose_in.header.frame_id) and tf_listener.frameExists(frame):
        tf_listener.waitForTransform(pose_in.header.frame_id, frame, pose_in.header.stamp, rospy.Duration(0.5))
        pose = tf_listener.transformPose(frame, pose_in)
        return pose
    return None


def cb_set_status(request) :
    global active
    result = SetShepherdingStatusResponse()
    if request.target_status is False :
        active = False
        result.result = "Shepherding disabled"
    else :
        if food is None :
            active = False
            result.result = "Shepherding not enabled, no food found"
        else :
            active = True
            result.result = "Shepherding enabled!"
    rospy.loginfo(result.result)
    return result


def cb_found_turtles(msg):
    global turtles
    current_turtles = []
    #rospy.loginfo("Previous turtles: %s"%str(turtles))
    
    for turtle in msg.turtles:
        current_turtles.append(turtle.name)
        if active and turtle.name not in turtles :
            bark_at(turtle)
    
    if active :
        turtles = current_turtles
        #rospy.loginfo("Barked at: %s"%str(turtles))

def cb_found_food(msg) :
    global food
    food = transform_pose(msg)
    rospy.loginfo("Found food!")


def bark_at(turtle) :
    msg = CommunicationProtocol()
    msg.sender = own_name
    msg.receiver = turtle.name
    msg.request = "mitro_message"
    msg.food_location = food
    msg.food_location.header.stamp = rospy.Time.now()
    turtle_pose = transform_pose(turtle.position)
    if turtle_pose is not None :
        msg.robot_location = turtle_pose
        comm_pub.publish(msg)
        rospy.loginfo("Barked at %s"%(turtle.name))


def main() :
    global tf_listener, comm_pub
    rospy.init_node('shepherding')
    
    tf_listener = tf.TransformListener()
    
    comm_pub = rospy.Publisher('/communication', CommunicationProtocol, queue_size=10)
    shep_pub = rospy.Publisher('/mitro_shepherd/status', Bool, queue_size=1)
    rospy.Subscriber('found_turtles', Turtles, cb_found_turtles)
    rospy.Subscriber('found_food', PoseStamped, cb_found_food)
    rospy.Service('mitro_shepherd/set_shepherding_status', SetShepherdingStatus, cb_set_status)
    
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        shep_pub.publish(active)
        r.sleep()


if __name__ == "__main__" :
    main()
