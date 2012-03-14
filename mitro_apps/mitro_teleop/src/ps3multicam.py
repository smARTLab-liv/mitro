#!/usr/bin/env python
import roslib; 
roslib.load_manifest('mitro_teleop')
import rospy
from sensor_msgs.msg import Joy

class MultiCam:
    filename = '/tmp/multicam-fifo'
    
    def __init__(self):
        self.view = 0
        self.idx = 0
        self.pressed = 0
        self.fd = open(self.filename, 'w')
        
    def callback(self, data):
        if data.buttons[self.idx] != self.pressed:
            self.pressed = data.buttons[self.idx]
            if not self.pressed:
                self.view = (self.view + 1) % 4
                rospy.loginfo("swuitching to view %d",self.view)
                self.fd.write(str(self.view + 1) + '\n')
                self.fd.flush()

if __name__ == '__main__':
    multicam = MultiCam()
    rospy.init_node('ps3multicam', anonymous=True)
    rospy.Subscriber("joy", Joy, multicam.callback)
    rospy.spin()
