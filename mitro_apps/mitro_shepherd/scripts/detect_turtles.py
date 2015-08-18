#!/usr/bin/env python
import rospy
import tf
import math
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, Quaternion
from swarming_turtles_msgs.msg import Turtle,Turtles

output_frame = '/map'

NUM_TURTLES = 15

class DetectTurtles:
    def __init__(self):
        self.tfListen = tf.TransformListener()
        self.turtle_pub = rospy.Publisher('found_turtles', Turtles, queue_size=10)
        rospy.sleep(0.5)
        rospy.Subscriber('ar_pose_marker_filtered', AlvarMarkers, self.cb_ar_marker)
       
        
    def transform_pose(self,pose_in):
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
            turtle_id = int(marker.id) - 100
            if turtle_id > NUM_TURTLES and turtle_id < 0 :
                continue
            id_str = "tb%02d" % turtle_id
            turtle = {}
            turtle['name'] = id_str
            turtle['pose'] = self.transform_pose(marker.pose)
            turtles.append(turtle)
        #print turtles
        self.publish_turtles(turtles)


    def publish_turtles(self, turtles):
        msg = Turtles()
        for t in turtles:
            turtle = Turtle()
            turtle.name = t['name']
            turtle.position = t['pose']
            msg.turtles.append(turtle)
        self.turtle_pub.publish(msg)

def main():
    rospy.init_node("detect_turtles")
    detect = DetectTurtles()
    rospy.spin()


if __name__ == '__main__':
    main()

