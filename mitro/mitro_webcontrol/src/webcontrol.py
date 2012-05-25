#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import argparse
import random
import os

import cherrypy

from ws4py.server.cherrypyserver import WebSocketPlugin, WebSocketTool
from ws4py.websocket import WebSocket
from ws4py.messaging import TextMessage

import Skype4Py
from Skype4Py import SkypeError

import roslib;
roslib.load_manifest('mitro_webcontrol')
import rospy

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Int16, Bool
from mitro_diagnostics.msg import SysInfo
from nav_msgs.msg import MapMetaData

from subprocess import Popen, PIPE

import tf

import json

skype_sequence = '''key F
'''




def keypress(sequence):
    """ This uses commandline tool xte to execute a keysequence. Used to fullscreen Skype video call window. """
    p = Popen(['xte'], stdin=PIPE)
    p.communicate(input=sequence)

class ChatWebSocketHandler(WebSocket):
    
    def __init__(self, *args, **kwargs):
        super(ChatWebSocketHandler, self).__init__(*args, **kwargs)

    def received_message(self, m):
        """ Handles Websocket messages. """
        if not m.is_text:
            return

        text = m.data

        if text == 'connect':
            response = TextMessage('connected')
            cherrypy.engine.publish('websocket-broadcast', response) # this proably shouldn't be a broadcast, just one connection to server
            
        if text.startswith('cmd:'):
            # command velocity protocol looks like:
            # cmd:[linear_speed],[angular_speed], where linear_ and angular_speed are floats
            r = text.split(':')
            cmdstr = str(r[1]);
            cmd = cmdstr.split(',')
            linear = float(cmd[0])
            angular = float(cmd[1])

            data = Twist()
            data.linear.x = linear
            data.angular.z = angular

            pub_twist.publish(data)

            #response = TextMessage('cmd twist: %f %f'%(linear, angular))
            #cherrypy.engine.publish('websocket-broadcast', response)


        if text.startswith('view:'):
            # command view switches view
            # protocol looks like:
            # view:[view_number] where view_number is 1-4
            r = text.split(':')
            view = int(r[1])
            if view < 5 and view > 0:
                pub_view.publish(view-1) # apparently the view subscriper likes only view_number between 0-3
                response = TextMessage('switched to view: %d'%view)
                cherrypy.engine.publish('websocket-broadcast', response)

        if text.startswith('skype:'):
            # skype command protocol:
            # skype:[contact_name]
            r = text.split(':')
            user = str(r[1])

            response = TextMessage('trying to call: %s'%user)
            cherrypy.engine.publish('websocket-broadcast', response)

            try:
                # Add a user (send request)
                uprofile = skype.User(Username=user)
                if uprofile.BuddyStatus != Skype4Py.budFriend:
                    uprofile.SetBuddyStatusPendingAuthorization('hello %s'%user) # TODO this could be a more descriptive message

                # Call the user
                a = skype.PlaceCall(user)

                t = rospy.get_time()
                while not a.Status == 'INPROGRESS': # wait until call is accepted by remote user
                    rospy.sleep(1)
                    if rospy.get_time() - t > 60: # wait a maximum of one minute 
                        break

                rospy.sleep(5)
                keypress(skype_sequence) # send fullscreen command to skype


            except SkypeError as details: # skype fucked up
                response = TextMessage('skype failed: %s'%details.args[1])
                cherrypy.engine.publish('websocket-broadcast', response)
                
            

    def closed(self, code, reason="A client left the room without a proper explanation."): # well that's clearly from the example
        cherrypy.engine.publish('websocket-broadcast', TextMessage(reason))

class Root(object):
    def __init__(self, host, port):
        self.host = host
        self.port = port

    @cherrypy.expose
    def index(self):
        """ this is definetely not just the index webpage """
        htmldoc = file('static/index.html', 'r').read()
        jsscript = file('static/script.js', 'r').read()

        return htmldoc%{'script': jsscript%{ 'host': self.host,
                                             'port': self.port}}

    @cherrypy.expose
    def ws(self):
        """ websocket handling """
        cherrypy.log("Handler created: %s" % repr(cherrypy.request.ws_handler))


status_msg = {}

def callback_relais(msg):
    global status_msg
    status_msg['relais'] = msg.data

def callback_runstop(msg):
    global status_msg
    status_msg['runstop'] = msg.data

def callback_hasgoal(msg):
    global status_msg
    status_msg['hasgoal'] = msg.data

def callback_assisted(msg):
    global status_msg
    status_msg['assisted'] = msg.data

def callback_mapmeta(msg):
    global mapmeta
    mapmeta = msg

from threading import Thread

class StatusThread(Thread):
    def __init__ (self):
        Thread.__init__(self)
        self.stopped = False
        self.listener = tf.TransformListener()
    
    def stop(self):
        self.stopped = True

    def run(self):
        global status_msg, mapmeta
        while not self.stopped:
            try:
                (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                trans = None

            if trans and mapmeta:
                (x, y, th) = trans
                status_msg['location'] = (int((x-mapmeta.origin.position.x)/mapmeta.resolution), 
                                          int((y-mapmeta.origin.position.y)/mapmeta.resolution))
                
            cherrypy.engine.publish('websocket-broadcast', TextMessage(json.dumps(status_msg)))
            rospy.sleep(1)


from cherrypy.process import plugins

class MyFeature(plugins.SimplePlugin):
    """A feature that does something."""

    def start(self):
        self.bus.log("Starting my feature")
        self.thread = StatusThread()
        self.thread.start()

    def stop(self):
        self.bus.log("Stopping my feature.")
        self.thread.stop()
        self.thread.join()

my_feature = MyFeature(cherrypy.engine)
my_feature.subscribe()


if __name__ == '__main__':

    #global fd

    #filename = '/tmp/multicam-fifo'
    #fd = open(filename, 'w');

    global skype
    # Create an instance of the Skype class.
    skype = Skype4Py.Skype(Transport='x11')
    # Connect the Skype object to the Skype client.
    skype.Attach()



    rospy.init_node('web_control', anonymous=False, disable_signals=True)
    global pub_twist, pub_view
    pub_twist = rospy.Publisher("cmd_twist_tele", Twist)
    pub_view = rospy.Publisher("/multicam/view", Int16)

    rospy.Subscriber("/relais", Bool, callback_relais)
    rospy.Subscriber("/runstop", Bool, callback_runstop)
    rospy.Subscriber("/assisted_drive/status", Bool, callback_assisted)
    rospy.Subscriber("/goal_planner/has_goal", Bool, callback_hasgoal)
    rospy.Subscriber("/map_metadata", MapMetaData, callback_mapmeta)


#    rospy.Subscriber("/sysinfo", SysInfo, callback_sysinfo)
#    rospy.Subscriber("/move_base/current_goal", PoseStamped, callback_goal)
#    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_amcl)

    parser = argparse.ArgumentParser(description='Echo CherryPy Server')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('-p', '--port', default=9000, type=int)
    args = parser.parse_args()

    cherrypy.config.update({'server.socket_host': args.host,
                            'server.socket_port': args.port,
                            'tools.staticdir.root': os.path.abspath(os.path.join(os.path.dirname(__file__), 'static'))})



    WebSocketPlugin(cherrypy.engine).subscribe()
    cherrypy.tools.websocket = WebSocketTool()

    cherrypy.quickstart(Root(args.host, args.port), '', config={
        '/ws': {
            'tools.websocket.on': True,
            'tools.websocket.handler_cls': ChatWebSocketHandler
            },
        '/static': {
              'tools.staticdir.on': True,
              'tools.staticdir.dir': ''
            }
        }
    )


        
        
        
