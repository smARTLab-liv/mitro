#!/usr/bin/env python
import web

import roslib
roslib.load_manifest("tele_remote")
import rospy
from geometry_msgs.msg import Twist

urls = (
    '/', 'index',
    '/move_base', 'move_base'
)
render = web.template.render('templates/')
app = web.application(urls, globals())

rospy.init_node('tele_remote_server')
vel_pub = rospy.Publisher('cmd_twist', Twist)

class index:
    def GET(self):
        return render.index()

class move_base:
    global vel_pub

    def POST(self):
        data = web.input()
        x = 0.01 * (int(data.x) - 100)
        y = -0.01 * (int(data.y) - 100)

        vx = 0.5 * y
        vw = 0.5 * x

        t = Twist()
        t.linear.x = vx
        t.angular.z = vw
        
        vel_pub.publish(t) 

if __name__ == "__main__":
    app.run()
