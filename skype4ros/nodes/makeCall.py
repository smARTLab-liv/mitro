#!/usr/bin/env python
import roslib; roslib.load_manifest('skype4ros')

from skype4ros.srv import *
import rospy

def handle_make_outgoing_call(req):
    print "Making a call to: [%s]"%(req.callee)
    return AddTwoIntsResponse(True)

def skype4ros_server():
    rospy.init_node('skype4ros_server')
    s = rospy.Service('add_two_ints', callTo, handle_make_outgoing_call)
    print "Ready to start making calls."
    rospy.spin()

if __name__ == "__main__":
    skype4ros_server()
