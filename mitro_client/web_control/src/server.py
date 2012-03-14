#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import random
import os

import cherrypy

from ws4py.server.cherrypyserver import WebSocketPlugin, WebSocketTool
from ws4py.websocket import WebSocket
from ws4py.messaging import TextMessage

class ChatWebSocketHandler(WebSocket):
    filename = '/tmp/multicam-fifo'
    
    def __init__(self, *args, **kwargs):
        super(ChatWebSocketHandler, self).__init__(*args, **kwargs)
        self.fd = open(self.filename, 'w');

    def received_message(self, m):
        if not m.is_text:
            return

        text = m.data

        if text == 'connect':
            response = TextMessage('connected')
            cherrypy.engine.publish('websocket-broadcast', response)
            

        if text.startswith('view:'):
            r = text.split(':')
            view = int(r[1])
            if view < 5 and view > 0:
                self.fd.write(str(view + 1) + '\n')
                self.fd.flush()
                response = TextMessage('switched to view: %d'%view)
                cherrypy.engine.publish('websocket-broadcast', response)

    def closed(self, code, reason="A client left the room without a proper explanation."):
        cherrypy.engine.publish('websocket-broadcast', TextMessage(reason))

class Root(object):
    def __init__(self, host, port):
        self.host = host
        self.port = port

    @cherrypy.expose
    def index(self):
        return """<html>
    <head>
      <script type='application/javascript' src='/js/jquery-1.6.2.min.js'></script>
      <script type='application/javascript'>
        $(document).ready(function() {

          websocket = 'ws://%(host)s:%(port)s/ws';
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
            $('#chat').val($('#chat').val() + 'bye bye...\\n');
            ws.close(1000, 'disconnect');
               
            if(!e) e = window.event;
            e.stopPropagation();
            e.preventDefault();
          };
          ws.onmessage = function (evt) {
             $('#chat').val($('#chat').val() + '<< ' + evt.data + '\\n');
          };
          ws.onclose = function(evt) {

             $('#chat').val($('#chat').val() + 'Connection closed by server: ' + evt.code + ' \"' + evt.reason + '\"\\n');  
          };

          $('#send').click(function() {
             // console.log($('#message').val());
             $('#chat').val($('#chat').val() + '>> ' + $('#message').val()  + '\\n');
             ws.send($('#message').val());
             $('#message').val("");
             return false;
          });
        });
      </script>
    </head>
    <body>
    <form action='#' id='chatform' method='get'>
      <textarea id='chat' cols='35' rows='10'></textarea>
      <br />
      <label for='message'>Command</label><input type='text' id='message' />
      <input id='send' type='submit' value='Send' />
      </form>
    </body>
    </html>
    """ % { 'host': self.host,
            'port': self.port}

    @cherrypy.expose
    def ws(self):
        cherrypy.log("Handler created: %s" % repr(cherrypy.request.ws_handler))

if __name__ == '__main__':
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
