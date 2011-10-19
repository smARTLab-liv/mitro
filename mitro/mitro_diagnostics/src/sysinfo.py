#!/usr/bin/env python

import roslib; roslib.load_manifest("mitro_diagnostics")
import rospy
from mitro_diagnostics.msg import SysInfo
from pythonwifi.iwlibs import Wireless
import psutil
import socket

def sysinfo():
    pub = rospy.Publisher('sysinfo', SysInfo)
    rospy.init_node('sysinfo')
    
    info_msg = SysInfo()
    info_msg.hostname = socket.gethostname()
    
    while not rospy.is_shutdown():
        info_msg.cpu_usage = psutil.cpu_percent()
        pub.publish(info_msg)
        
if __name__ == '__main__':
    try:
        sysinfo()
    except rospy.ROSInterruptException: pass
