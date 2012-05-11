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
roslib.load_manifest('geometry_msgs')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

from subprocess import Popen, PIPE

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

            response = TextMessage('cmd twist: %f %f'%(linear, angular))
            cherrypy.engine.publish('websocket-broadcast', response)


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
        """ this is just the index webpage """
        return """<html>
    <head>
      <script type='application/javascript' src='/js/jquery-1.6.2.min.js'></script>
      <script type='application/javascript'>
        function send_cmds() {
          if (keys[0] | keys[1] | keys[2] | keys[3]) {
            linear = 0;
            angular = 0;
            if (keys[0] & !keys[1]) {
              linear = 0.7;
            }
            if (keys[1] & !keys[0]) {
              linear = -0.15;
            }
            if (keys[2] & !keys[3]) {
              angular = 1;
            }
            if (keys[3] & !keys[2]) {
              angular = -1;
            }
            ws.send('cmd:' + linear + ',' + angular);
          } 
          setTimeout("send_cmds()", 100);
        };          

        $(document).ready(function() {
          websocket = 'ws://' + window.location.host + '/ws';
          // websocket = 'ws://%(host)s:%(port)s/ws';
          if (window.WebSocket) {
            ws = new WebSocket(websocket);
          }
          else if (window.MozWebSocket) {
            ws = MozWebSocket(websocket);
          }
          else {
            console.log('WebSocket Not Supported');
            return;
          }

          ws.onopen = function() {
             ws.send("connect");
          };
          window.onbeforeunload = function(e) {
            $('#chat').val('bye bye...\\n' + $('#chat').val());
            ws.close(1000, 'disconnect');
               
            if(!e) e = window.event;
            e.stopPropagation();
            e.preventDefault();
          };
          ws.onmessage = function (evt) {
             $('#chat').val('<< ' + evt.data + '\\n' + $('#chat').val());
          };
          ws.onclose = function(evt) {

             $('#chat').val($('#chat').val() + 'Connection closed by server: ' + evt.code + ' \"' + evt.reason + '\"\\n');  
          };

          $('#skype').click(function() {
             ws.send('skype:' + $('#skypeuser').val());
             return false;
          });

          $('#view1').click(function() {
             ws.send('view:1');
             return false;
          });

          $('#view2').click(function() {
             ws.send('view:2');
             return false;
          });

          $('#view3').click(function() {
             ws.send('view:3');
             return false;
          });

          $('#view4').click(function() {
             ws.send('view:4');
             return false;
          });


          keys = new Array(0, 0, 0, 0);

          document.onkeydown = function(e){
            keyCode = ('which' in event) ? event.which : event.keyCode;
            switch(keyCode) {
              case 38:
                // up
                keys[0] = 1;
                break;
              case 40:
                // down
                keys[1] = 1;
                break;
              case 37:
                // left
                keys[2] = 1;
                break;
              case 39:
                // right
                keys[3] = 1;
                break;
            };
          };

          document.onkeyup = function(e){
            keyCode = ('which' in event) ? event.which : event.keyCode;
            switch(keyCode) {
              case 38:
                // up
                keys[0] = 0;
                break;
              case 40:
                // down
                keys[1] = 0;
                break;
              case 37:
                // left
                keys[2] = 0;
                break;
              case 39:
                // right
                keys[3] = 0;
                break;
            };
          };

          document.onblur = function(e) {
            keys = Array(0, 0, 0, 0);
          };

          setTimeout("send_cmds()", 100);

        });
      </script>
    </head>
    <body>
    <form action='#' id='chatform' method='get'>
      <textarea id='chat' cols='35' rows='10'></textarea>
      <br />
      <input id='skypeuser' type='text'/>
      <input id='skype' type='submit' value='Call' />
      <br />
      <input id='view1' type='submit' value='View 1' />
      <input id='view2' type='submit' value='View 2' />
      <input id='view3' type='submit' value='View 3' />
      <input id='view4' type='submit' value='View 4' />
      </form>
    </body>
    </html>
    """ % { 'host': self.host,
            'port': self.port}

    @cherrypy.expose
    def ws(self):
        """ websocket handling """
        cherrypy.log("Handler created: %s" % repr(cherrypy.request.ws_handler))

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
        '/js': {
              'tools.staticdir.on': True,
              'tools.staticdir.dir': 'js'
            }
        }
    )
