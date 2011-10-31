#!/usr/bin/env python

import roslib
roslib.load_manifest('tele_controllers')
import rospy
from geometry_msgs.msg import Twist
from tele_msgs.msg import VelocityCommand
import math
import sys
  
class BaseController:
    def __init__(self, params):    
        self.vel_rot = 0
        self.vel_linear = 0
        
        self.wheel_radius = params['wheel_radius']
        self.wheel_basis = params['wheel_basis']
        
        self.last_time = rospy.Time.now()

        self.pub = rospy.Publisher('cmd_vel', VelocityCommand)
        rospy.Subscriber('cmd_twist', Twist, self.cmd_vel_cb)

        
    def cmd_vel_cb(self, msg):
        self.vel_rot = msg.angular.z
        self.vel_linear = msg.linear.x
        self.last_time = rospy.Time.now()
        
    def update(self):    
        vc = VelocityCommand()
        if (rospy.Time.now() - self.last_time).to_sec() < 0.5:
            vc.velocity_left = self.vel_linear / self.wheel_radius - 0.5 * self.vel_rot * self.wheel_basis / self.wheel_radius 
            vc.velocity_right = self.vel_linear / self.wheel_radius + 0.5 * self.vel_rot * self.wheel_basis / self.wheel_radius
            vc.velocity_right *= 1/(2 * math.pi) * 36.0/2.0
            vc.velocity_left *= 1/(2 * math.pi) * 36.0/2.0
        else:
            rospy.logerr('Did not receive Twist message in time!')
        self.pub.publish(vc)
        
def main():        
    rospy.init_node('tele_base_controller')
    ns = 'tele_base_controller'
    try:
        params = rospy.get_param(ns)
    except KeyError:
        rospy.logerr('Parameters not loaded, namespace = %s'%ns)
        sys.exit(1)

    base_controller = BaseController(params)
    while not rospy.is_shutdown():    
        base_controller.update()
        rospy.sleep(0.01)

if __name__ == '__main__':
    main()
