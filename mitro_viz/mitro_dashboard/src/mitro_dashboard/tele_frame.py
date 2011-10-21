# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# based on turtelebod_dashboard

import roslib
roslib.load_manifest('mitro_dashboard')

import wx
import wx.aui
import wx.py.shell
import rxtools
import rxtools.cppwidgets as rxtools

import std_msgs.msg

import rospy
from roslib import rosenv

import os
import threading

from status_control import StatusControl
from battery_state_control import BatteryStateControl
from rosout_frame import RosoutFrame

class TeleFrame(wx.Frame):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    
    def __init__(self, parent, id=wx.ID_ANY, title='MITRO Dashboard', pos=wx.DefaultPosition, size=(400, 50), style=wx.CAPTION|wx.CLOSE_BOX|wx.STAY_ON_TOP):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        
        wx.InitAllImageHandlers()
        
        self.SetBackgroundColour(wx.Colour(242,241,240,255))

        master = rosenv.get_master_uri()
        robot = rospy.get_namespace()
        if len(robot) > 1:
            robot = robot[1:robot.rfind('/')]
        else:    
            robot = master[7:master.rfind(':')]
        self.SetTitle('MITRO (%s)'%robot)

        rospy.init_node('tele_dashboard', anonymous=True)
        try:
            getattr(rxtools, "initRoscpp")
            rxtools.initRoscpp("tele_dashboard_cpp", anonymous=True)
        except AttributeError:
            pass
        
       
        icons_path = os.path.join(roslib.packages.get_pkg_dir('tele_dashboard'), "icons/")
        
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)
        
        box = wx.StaticBox(self, wx.ID_ANY, "ROS")
        #box.SetBackgroundColour(wx.Colour(200,0,0))

        static_sizer = wx.StaticBoxSizer(box, wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # Rosout
        self._rosout_button = StatusControl(self, wx.ID_ANY, icons_path, "rosout", True)
        self._rosout_button.SetToolTip(wx.ToolTip("Rosout"))
        static_sizer.Add(self._rosout_button, 0)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Runstops"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)

        # Relais
        self._relais_button = StatusControl(self, wx.ID_ANY, icons_path, "motor", True)
        self._relais_button.SetToolTip(wx.ToolTip("Relais"))
        static_sizer.Add(self._relais_button, 0)
        self._relais_button.Bind(wx.EVT_BUTTON, self.on_relais_clicked)
        
        # Run-stop
        self._runstop_ctrl = StatusControl(self, wx.ID_ANY, icons_path, "runstop", False)
        self._runstop_ctrl.SetToolTip(wx.ToolTip("Runstop: Unknown"))
        static_sizer.Add(self._runstop_ctrl, 0)

                                        
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Battery"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # Battery State
        self._battery_state_ctrl = BatteryStateControl(self, wx.ID_ANY, icons_path)
        self._battery_state_ctrl.SetToolTip(wx.ToolTip("Battery: Stale"))
        static_sizer.Add(self._battery_state_ctrl, 1, wx.EXPAND)
        
        self._config = wx.Config("tele_dashboard")
        
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
        self.Layout()
        self.Fit()
        
        self._rosout_frame = RosoutFrame(self, wx.ID_ANY, "Rosout")
        self._rosout_frame.Hide()
        self._rosout_frame.Center()
        self._rosout_button.Bind(wx.EVT_BUTTON, self.on_rosout_clicked)
        
        self.load_config()
        
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self._timer.Start(500)
    
        self._last_battery_message_time = 0.0
        self._last_relais_message_time = 0.0
        self._last_runstop_message_time = 0.0

        self._relais = False

        self._sub_battery_voltage = rospy.Subscriber('battery_voltage', std_msgs.msg.Float32, self.cb_battery_voltage)
        rospy.loginfo(("subscribed to topic:=%s")%self._sub_battery_voltage.name)
        self._sub_relais = rospy.Subscriber('relais', std_msgs.msg.Bool, self.cb_relais)
        rospy.loginfo(("subscribed to topic:=%s")%self._sub_relais.name)
        self._sub_runstop = rospy.Subscriber('runstop', std_msgs.msg.Bool, self.cb_runstop)
        rospy.loginfo(("subscribed to topic:=%s")%self._sub_runstop.name)

        self._pub_relais = rospy.Publisher('cmd_relais', std_msgs.msg.Bool)


    def __del__(self):
        self._sub_battery_voltage.unregister()
        self._sub_relais.unregister()
        self._sub_runstop.unregister()
        
    def on_timer(self, evt):
      self.update_rosout()
      
      if (rospy.get_time() - self._last_battery_message_time > 5.0):
          self._battery_state_ctrl.set_stale()
          self._battery_state_ctrl.SetToolTip(wx.ToolTip("No message received on \"%s\" in the last 5 seconds"%self._sub_battery_voltage.name))

      if (rospy.get_time() - self._last_relais_message_time > 5.0):
          self._relais_button.set_stale()
          self._relais_button.SetToolTip(wx.ToolTip("No message received on \"%s\" in the last 5 seconds"%self._sub_relais.name))
     
      if (rospy.get_time() - self._last_runstop_message_time > 5.0):
          self._runstop_ctrl.set_stale()
          self._runstop_ctrl.SetToolTip(wx.ToolTip("No message received on \"%s\" in the last 5 seconds"%self._sub_runstop.name))
   
      if (rospy.is_shutdown()):
        self.Close()
        
    def on_rosout_clicked(self, evt):
      self._rosout_frame.Show()
      self._rosout_frame.Raise()
      
    def on_relais_clicked(self, evt):
        self._pub_relais.publish(not self._relais)

    def cb_battery_voltage(self, msg):
        wx.CallAfter(self.update_battery_voltage, msg)

    def update_battery_voltage(self, msg):
        self._last_battery_message_time = rospy.get_time()
        self._battery_state_ctrl.set_state(msg)

    def cb_relais(self, msg):
        wx.CallAfter(self.update_relais, msg)

    def update_relais(self, msg):
        self._last_relais_message_time = rospy.get_time()
        self._relais = msg.data
        if self._relais:
            self._relais_button.set_ok()
        else:
            self._relais_button.set_error()

    def cb_runstop(self, msg):
        wx.CallAfter(self.update_runstop, msg)

    def update_runstop(self, msg):
        self._last_runstop_message_time = rospy.get_time()
        if not msg.data:
            self._runstop_ctrl.SetToolTip(wx.ToolTip("Runstop: on"))
            self._runstop_ctrl.set_ok()
        else:
            self._runstop_ctrl.SetToolTip(wx.ToolTip("Runstop: pressed"))
            self._runstop_ctrl.set_error()
          
    def update_rosout(self):
      summary_dur = 30.0
      if (rospy.get_time() < 30.0):
          summary_dur = rospy.get_time() - 1.0
          
      if (summary_dur < 0):
          summary_dur = 0.0
    
      summary = self._rosout_frame.get_panel().getMessageSummary(summary_dur)
      
      if (summary.fatal or summary.error):
        self._rosout_button.set_error()
      elif (summary.warn):
        self._rosout_button.set_warn()
      else:
        self._rosout_button.set_ok()
        
        
      tooltip = ""
      if (summary.fatal):
        tooltip += "\nFatal: %s"%(summary.fatal)
      if (summary.error):
        tooltip += "\nError: %s"%(summary.error)
      if (summary.warn):
        tooltip += "\nWarn: %s"%(summary.warn)
      if (summary.info):
        tooltip += "\nInfo: %s"%(summary.info)
      if (summary.debug):
        tooltip += "\nDebug: %s"%(summary.debug)
      
      if (len(tooltip) == 0):
        tooltip = "Rosout: no recent activity"
      else:
        tooltip = "Rosout: recent activity:" + tooltip
    
      if (tooltip != self._rosout_button.GetToolTip().GetTip()):
          self._rosout_button.SetToolTip(wx.ToolTip(tooltip))
        
    def load_config(self):
      # Load our window options
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      if (self._config.HasEntry(self._CONFIG_WINDOW_X)):
          x = self._config.ReadInt(self._CONFIG_WINDOW_X)
      if (self._config.HasEntry(self._CONFIG_WINDOW_Y)):
          y = self._config.ReadInt(self._CONFIG_WINDOW_Y)
      
      self.SetPosition((x, y))
      self.SetSize((width, height))
        
    def save_config(self):
      config = self._config
      
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      config.WriteInt(self._CONFIG_WINDOW_X, x)
      config.WriteInt(self._CONFIG_WINDOW_Y, y)
      
      config.Flush()
        
    def on_close(self, event):
      self.save_config()
      
      self.Destroy()
      
