#!/usr/bin/env python
import roslib
roslib.load_manifest('tele_imu')  
import rospy

import math
import serial
import threading
import sys

class SerialInput(threading.Thread):

    def __init__(self, com):
        threading.Thread.__init__(self)
        self.com = com
        if not self.com.isOpen():
            rospy.logerr('serial port is not open yet!')
            sys.exit(-1)
        self.linebuffer = []
            
    def run(self):
        while not rospy.is_shutdown():
            if (self.com.inWaiting > 100):
                self.linebuffer.append(self.com.readline())

    def numLines(self):
	return len(self.linebuffer)

    def getLine(self):
        if len(self.linebuffer) > 0:
            return self.linebuffer.pop(0)
        else:
            return None


class IMU():

    def __init__(self):
	pass

    def connect(self):    
        rospy.loginfo('try to connect to arduino')
        try:
            self.com = serial.Serial(
                port = '/dev/ttyUSB0', 
                bytesize = 8, 
                baudrate = 115200,
                timeout = 5)
        except serial.serialutil.SerialException:
            rospy.logerr('arduino appears to be offline')
            sys.exit(1)
        rospy.loginfo('connected to arduino')

        rospy.sleep(0.5)

        # start non-blocking serial input
        self.serialinput = SerialInput(self.com)
        self.serialinput.start()


    def disconnect(self):
        self.com.close()


    def update(self):

        line = self.serialinput.getLine()
        if (line == None) :
            rospy.sleep(0.01)
            return
	    
	    
        for c in line :
            print hex(ord(c))[2:], " ",
	print "bytes ", self.com.inWaiting(), "lines ", self.serialinput.numLines() 
        
        if len(line) != 13 :
            print "oh noooooes, wrong imputz"
	    print "bytes ", self.com.inWaiting(), "lines ", self.serialinput.numLines() 
            return            
	   

    

if __name__ == "__main__":
    rospy.init_node('tele_imu')
#    ns = 'tele_imu'
#    try:
#       params = rospy.get_param(ns)
#    except KeyError:
#        rospy.logerr('Parameters not loaded, namespace = %s'%ns)
#        sys.exit(1)

    imu = IMU()
    imu.connect()

    rospy.sleep(0.1)
    imu.serialinput.getLine()

    r = rospy.Rate(500)
    while not rospy.is_shutdown():
        imu.update()
        r.sleep()

    imu.disconnect()
