#!/usr/bin/env python
import roslib; roslib.load_manifest('skype4ros')
import rospy
import time
import Skype4Py
import os
from skype4ros.msg import callTo
from skype4ros.msg import acceptVid
from std_msgs.msg import String

s = Skype4Py.Skype(Transport='x11')
# s = Skype4Py.Skype()
call = 1
video = 0

def handle_makeCall(callTo):
    rospy.loginfo(rospy.get_name()+" Making a call to: %s",callTo.target)
    global s
    global call
    global pub
    if s.AttachmentStatus == -1:
        s.Attach()
    # place the call to the target
    if s.AttachmentStatus == 0:
        call = s.PlaceCall(callTo.target)
        sleeptimer = (0.5)
        str = ""
        printed = False
        while 1:
            time.sleep(sleeptimer)
            rospy.loginfo(rospy.get_name()+" Status of the call: %s",call.Status)
            if call.Status == Skype4Py.clsInProgress:
                if printed == False:
                    rospy.loginfo(rospy.get_name()+" Conversation in progress")
                    str = "Conversation in progress"
                    printed = True
                sleeptimer = 10
            if call.Status == Skype4Py.clsFinished:
                str = "Conversation Finished"
                break
            if call.Status == Skype4Py.clsCancelled:
                str = "Connection cancelled on client side"
                break
            if call.Status == Skype4Py.clsFailed:
                str = "Connection Failed on server side"
                break
            pub.publish(String(str))
        pub.publish(String(str))#only used in case a break condition is hit.

def handle_acceptVid(acceptVid):
    global s
    global call
    global pub
    str = ""
    rospy.loginfo(rospy.get_name()+" Triggered AcceptVid")
    #if acceptVid.allow == 1 and video == 0:
    if acceptVid.allow == 1 and call.VideoStatus != Skype4Py.vssRunning:
        call.StartVideoReceive()
        try:
            call.StartVideoSend()
            time.sleep(5)
            os.system("xsendkeycode 41 1")#Based on physical pos on keyboard
            os.system("xsendkeycode 41 0")#Presses and releases the 'f' key (1 for press and 0 for release)
            str="Video Feed succesfully accepted"
        except:
            rospy.loginfo(rospy.get_name()+" Error sending videostream.")
            str="Video Feed failed"
        video = 1
    pub.publish(String(str))
    

def listener():
    global pub
    #rospy.init_node('skype4ros', anonymous=True)
    rospy.init_node('skype4ros')
    rospy.Subscriber("/Skype", callTo, handle_makeCall)
    rospy.Subscriber("/SkypeVid", acceptVid, handle_acceptVid)
    str = "Skype4Ros activated"
    pub.publish(String(str))
    rospy.spin()

if __name__ == '__main__':
    listener()
