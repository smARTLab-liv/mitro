#!/usr/bin/env python
import roslib
roslib.load_manifest('tele_controllers')  
import rospy

import math
import serial
import threading
import sys
from geometry_msgs.msg import Twist
from tele_msgs.msg import JointStates


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
            self.linebuffer.append(self.com.readline())
            
    def getLine(self):
        if len(self.linebuffer) > 0:
            return self.linebuffer.pop(0)
        else:
            return None


class Controller():

    def __init__(self, params):
        self.cnt = 0
        self.freq = 20
        self.max_speed = 50
        
        self.vel_rot = 0
        self.vel_linear = 0
        
        self.wheel_radius = params['wheel_radius']
        self.wheel_basis = params['wheel_basis']
        
        self.last_time = rospy.Time.now()

        self.pub = rospy.Publisher('joint_states', JointStates)
        rospy.Subscriber('cmd_twist', Twist, self.cmd_vel_cb)


    def cmd_vel_cb(self, msg):
        self.vel_rot = msg.angular.z
        self.vel_linear = msg.linear.x
        self.last_time = rospy.Time.now()


    def connect(self):    
        rospy.loginfo('try to connect to propeller')
        try:
            self.com = serial.Serial(
                port = '/dev/ttyUSB0', 
                bytesize = 8, 
                baudrate = 115200,
                timeout = 5)
        except serial.serialutil.SerialException:
            rospy.logerr('propeller appears to be offline')
            sys.exit(1)
        rospy.loginfo('connected to propeller')

        rospy.sleep(0.5)
	
	# switch on relais and ping sensors
	self.com.write('S1;')

        # start non-blocking serial input
        self.serialinput = SerialInput(self.com)
        self.serialinput.start()


    def disconnect(self):
        # first switch off relais and ping sensors
        self.com.write('S0;')
        self.com.close()
        rospy.loginfo('disconnected from propeller')


    def update(self):
        
        if (rospy.Time.now() - self.last_time).to_sec() < 0.5:
            velocity_left = self.vel_linear / self.wheel_radius - 0.5 * self.vel_rot * self.wheel_basis / self.wheel_radius 
            velocity_right = self.vel_linear / self.wheel_radius + 0.5 * self.vel_rot * self.wheel_basis / self.wheel_radius
            velocity_right *= 1/(2 * math.pi) * 36.0/2.0
            velocity_left *= 1/(2 * math.pi) * 36.0/2.0
            # ensure correct turning angle when speed is too high
            
            self.com.write('R'+str(int(velocity_right))+';L'+str(int(velocity_left))+';')
        else:
            rospy.logerr('Did not receive Twist message in time!')

        while 1 :
            line = self.serialinput.getLine()
            if (line == None) :
                break        
            line = line.strip()
            data = line.split(',')
            # print data
            
            if data[0] == '#S':
                # status message, skip for now
                if data[1] == '0':
                    print 'relais switched off..'
            elif data[0] == '#P':
                # ping sensors
                #print 'ping sensors', data[1:-1]
                pass
            elif data[0] == '#E':
                # encoders
                # print 'encoders', data[1:-1]
                msg = JointStates()
                msg.velocity_right = float(data[1])
                msg.velocity_left = float(data[2])
                msg.position_right = int(data[3])
                msg.position_left = int(data[4])
                self.pub.publish(msg)
            elif data[0] == '#D':
                # debug
                #print 'DEBUG', data[1:-1]
                rospy.logdebug('DEBUG %s', data[1:-1])
    

if __name__ == "__main__":
    rospy.init_node('base_controller')
    ns = 'tele_base_controller'
    try:
        params = rospy.get_param(ns)
    except KeyError:
        rospy.logerr('Parameters not loaded, namespace = %s'%ns)
        sys.exit(1)

    controller = Controller(params)
    controller.connect()

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        controller.update()
        r.sleep()

    controller.disconnect()
