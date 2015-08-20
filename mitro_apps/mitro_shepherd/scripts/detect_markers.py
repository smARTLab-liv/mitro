#!/usr/bin/env python
import rospy
import tf
import math
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, Quaternion
from swarming_turtles_msgs.msg import Turtle,Turtles

HIVE_FRAME = "/hive"
OUTPUT_FRAME = '/map'

TURTLES = range(1,16)
FOOD = 198
HIVE = 199

RATE = 20

transform = {}

def quat_msg_to_array(quat):
    return [quat.x, quat.y, quat.z, quat.w]


class DetectMarkers:
    def __init__(self):
        self.tfListen = tf.TransformListener()
        self.turtle_pub = rospy.Publisher('found_turtles', Turtles, queue_size=10)
        self.food_pub = rospy.Publisher('found_food', PoseStamped, queue_size=10)
        rospy.sleep(0.5)
        rospy.Subscriber('ar_pose_marker_filtered', AlvarMarkers, self.cb_ar_marker)
       
        
    def transform_pose(self,pose_in,output_frame=OUTPUT_FRAME):
        if pose_in.header.frame_id == output_frame:
            return pose_in
        
        if self.tfListen.frameExists(output_frame) and self.tfListen.frameExists(pose_in.header.frame_id):
            time = self.tfListen.getLatestCommonTime(pose_in.header.frame_id, output_frame)
            pose_in.header.stamp = time
            pose = self.tfListen.transformPose(output_frame, pose_in)
            return pose
        return None


    def cb_ar_marker(self, msg):
        turtles = []
        
        for marker in msg.markers: #check all markers
            marker_id = marker.id - 100
            if marker_id in TURTLES :
                id_str = "tb%02d" % marker_id
                turtle = {}
                turtle['name'] = id_str
                turtle['pose'] = self.transform_pose(marker.pose)
                turtles.append(turtle)
            elif marker_id == HIVE - 100 :
                self.update_tf(marker.pose)
            elif marker_id == FOOD - 100 :
                pose = PoseStamped()
                pose = marker.pose
                pose.header = msg.header
                self.food_pub.publish(self.transform_pose(pose))
        
        #print turtles
        self.publish_turtles(turtles)


    def update_tf(self, pose) :
        global transform
        transform['pose'] = (pose.pose.position.x, pose.pose.position.y, 0)
        transform['quat'] = tuple(quat_msg_to_array(pose.pose.orientation))
        transform['stamp'] = rospy.Time.now()


    def publish_turtles(self, turtles):
        msg = Turtles()
        for t in turtles:
            turtle = Turtle()
            turtle.name = t['name']
            turtle.position = t['pose']
            msg.turtles.append(turtle)
        self.turtle_pub.publish(msg)

def main():
    global transform
    rospy.init_node("detect_markers")
    tf_br = tf.TransformBroadcaster()
    transform['pose'] = (0, 0, 0)
    transform['quat'] = (0, 0, 0, 1)
    transform['stamp'] = rospy.Time.now()
    
    detect = DetectMarkers()
    
    r = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        tf_br.sendTransform(transform['pose'],
                            transform['quat'],
                            rospy.Time.now(),
                            HIVE_FRAME,
                            OUTPUT_FRAME)
        r.sleep()


if __name__ == '__main__':
    main()

