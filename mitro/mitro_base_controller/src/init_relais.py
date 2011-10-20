#!/usr/bin/env python

import roslib
roslib.load_manifest('mitro_base_controller')
import rospy
from std_msgs.msg import Bool
import sys

class RelaisInit:
	
	def __init__(self): 
		self.trying = False
		self.quit = False
		self.cmd_rel_pub = rospy.Publisher('cmd_relais', Bool)
		rospy.Subscriber('relais', Bool, self.relais_cb)
		self.main()

	def main(self):
		while not rospy.is_shutdown() and not self.quit:
			rospy.sleep(0.01)
		sys.exit(0)

	def relais_cb(self, msg):
		if not self.trying and not msg.data:
			rospy.loginfo('Arduino is online, trying to switch on relais..')
			rel_msg = Bool()
			rel_msg.data = True
			self.cmd_rel_pub.publish(rel_msg)
			self.trying = True
		elif not self.trying and msg.data:
			rospy.loginfo('Relais already switched on, exiting.')
			self.quit = True
		elif self.trying and not msg.data:
			rospy.loginfo('Still waiting for relais')
		elif self.trying and msg.data:
			rospy.loginfo('Relais switched on, exiting!')
			self.quit = True
		
def main():        
	rospy.init_node('init_relais')
	relais_init = RelaisInit()		
		
if __name__ == '__main__':
	main()
 
