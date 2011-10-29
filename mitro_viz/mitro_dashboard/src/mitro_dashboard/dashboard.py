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

import math

import wx
import wx.aui
import wx.py.shell
import rxtools
import rxtools.cppwidgets as rxtools

import std_msgs.msg
import mitro_diagnostics.msg

import rospy
from roslib import rosenv

import os
import threading

from status_control import StatusControl
from battery_state_control import BatteryStateControl
from rosout_frame import RosoutFrame

class Dashboard(wx.Frame):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    
    def __init__(self, parent, id=wx.ID_ANY, title='MITRO Dashboard', pos=wx.DefaultPosition, size=(400, 50), style=wx.CAPTION|wx.CLOSE_BOX|wx.STAY_ON_TOP):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        
        wx.InitAllImageHandlers()
        
        self.SetBackgroundColour(wx.Colour(242,241,240,255))

        # TODO: get from param server
        self._robot_hostname = "bob"
        self._laptop_hostname = "MacBook-Daan"

        self.SetTitle('MITRO (%s)'%self._robot_hostname)

        rospy.init_node('mitro_dashboard', anonymous=True)
        try:
            getattr(rxtools, "initRoscpp")
            rxtools.initRoscpp("mitro_dashboard_cpp", anonymous=True)
        except AttributeError:
            pass
               
        icons_path = os.path.join(roslib.packages.get_pkg_dir('mitro_dashboard'), "icons/")
        
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)


        # ROS
        box = wx.StaticBox(self, wx.ID_ANY, "ROS")
        static_sizer = wx.StaticBoxSizer(box, wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # Rosout
        self._rosout_button = StatusControl(self, wx.ID_ANY, icons_path, "rosout", True)
        self._rosout_button.SetToolTip(wx.ToolTip("Rosout"))
        static_sizer.Add(self._rosout_button, 0)




        # Robot
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Robot"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)

        # Relais
        self._relais_ctrl = StatusControl(self, wx.ID_ANY, icons_path, "motor", True)
        self._relais_ctrl.SetToolTip(wx.ToolTip("Relais"))
        static_sizer.Add(self._relais_ctrl, 0)
        self._relais_ctrl.Bind(wx.EVT_BUTTON, self.on_relais_clicked)
        
        # Run-stop
        self._runstop_ctrl = StatusControl(self, wx.ID_ANY, icons_path, "runstop", False)
        self._runstop_ctrl.SetToolTip(wx.ToolTip("Runstop: Unknown"))
        static_sizer.Add(self._runstop_ctrl, 0)

        # Wireless run-stop
        self._runstop_wireless_ctrl = StatusControl(self, wx.ID_ANY, icons_path, "runstop-wireless", False)
        self._runstop_wireless_ctrl.SetToolTip(wx.ToolTip("Wireless runstop: Unknown"))
        static_sizer.Add(self._runstop_wireless_ctrl, 0)


                                        
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Battery"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # Battery
        self._robot_battery_ctrl = BatteryStateControl(self, wx.ID_ANY, icons_path)
        self._robot_battery_ctrl.SetToolTip(wx.ToolTip("Battery: Stale"))
        static_sizer.Add(self._robot_battery_ctrl, 1, wx.EXPAND)


        # Laptop                        
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Laptop"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        self._laptop_battery_ctrl = BatteryStateControl(self, wx.ID_ANY, icons_path)
        self._laptop_battery_ctrl.SetToolTip(wx.ToolTip("Laptop battery: Stale"))
        static_sizer.Add(self._laptop_battery_ctrl, 1, wx.EXPAND)
        

        # Apps                        
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Apps"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)

        # Autonomous navigation
        self._goal_ctrl = StatusControl(self, wx.ID_ANY, icons_path, "motor", True)
        self._goal_ctrl.SetToolTip(wx.ToolTip("Autonomous navigation"))
        static_sizer.Add(self._goal_ctrl, 0)
        self._goal_ctrl.Bind(wx.EVT_BUTTON, self.on_nav_clicked)


        # Take me home
        self._home_ctrl = StatusControl(self, wx.ID_ANY, icons_path, "motor", True)
        self._home_ctrl.SetToolTip(wx.ToolTip("Take me home"))
        static_sizer.Add(self._home_ctrl, 0)
        # self._home_ctrl.Bind(wx.EVT_BUTTON, self.on_home_clicked)

        # Assisted teleop
        self._teleop_ctrl = StatusControl(self, wx.ID_ANY, icons_path, "motor", True)
        self._teleop_ctrl.SetToolTip(wx.ToolTip("Assisted teleop"))
        static_sizer.Add(self._teleop_ctrl, 0)
        # self._teleop_ctrl.Bind(wx.EVT_BUTTON, self.on_teleop_clicked)


        self._config = wx.Config("mitro_dashboard")
        
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


        self._last_robot_message = 0.0
        self._last_laptop_message = 0.0
        self._last_relais_message = 0.0
        self._last_runstop_message = 0.0
        self._last_runstop_wireless_message = 0.0
        self._last_goal_message = 0.0

        self._sub_runstop = rospy.Subscriber('runstop', std_msgs.msg.Bool, self.cb_runstop)
        self._sub_runstop_wireless = rospy.Subscriber('runstop_wireless', std_msgs.msg.Bool, self.cb_runstop_wireless)
        self._sub_relais = rospy.Subscriber('relais', std_msgs.msg.Bool, self.cb_relais)
        self._sub_sysinfo = rospy.Subscriber('sysinfo', mitro_diagnostics.msg.SysInfo, self.cb_sysinfo)
        self._pub_relais = rospy.Publisher('cmd_relais', std_msgs.msg.Bool)

        GOAL_TOPIC = "/goal_planner/has_goal"
        GOAL_CANCEL_TOPIC = "/goal_planner/cancel"
        self._sub_goal = rospy.Subscriber(GOAL_TOPIC, std_msgs.msg.Bool, self.cb_goal)
        self._pub_goal = rospy.Publisher(GOAL_CANCEL_TOPIC, std_msgs.msg.Bool)


    def __del__(self):
        self._sub_runstop.unregister()
        self._sub_runstop_wireless.unregister()
        self._sub_relais.unregister()
        self._sub_sysinfo.unregister()
        self._sub_goal.unregister()
        self._pub_goal.unregister()
        self._pub_relais.unregister()

        
    def on_timer(self, evt):
      self.update_rosout()

      if (rospy.get_time() - self._last_relais_message > 5.0):
          self._relais_ctrl.set_stale()
          self._relais_ctrl.SetToolTip(wx.ToolTip("No message received on \"%\
s\" in the last 5 seconds"%self._sub_relais.name))

      if (rospy.get_time() - self._last_runstop_message > 5.0):
          self._runstop_ctrl.set_stale()
          self._runstop_ctrl.SetToolTip(wx.ToolTip("No message received on \"%s\
\" in the last 5 seconds"%self._sub_runstop.name))

      if (rospy.get_time() - self._last_runstop_wireless_message > 5.0):
          self._runstop_wireless_ctrl.set_stale()
          self._runstop_wireless_ctrl.SetToolTip(wx.ToolTip("No message received on \"%s\
\" in the last 5 seconds"%self._sub_runstop_wireless.name))

      if (rospy.get_time() - self._last_goal_message > 5.0):
          self._goal_ctrl.set_stale()
          self._goal_ctrl.SetToolTip(wx.ToolTip("No message received on \"%s\
\" in the last 5 seconds"%self._sub_goal.name))


      if (rospy.get_time() - self._last_robot_message > 5.0):
          msg ='No message received in the last 5 sec from "%s"'%self._robot_hostname
          self._robot_battery_ctrl.set_stale()
          self._robot_battery_ctrl.SetToolTip(wx.ToolTip(msg))

      if (rospy.get_time() - self._last_laptop_message > 5.0):
          msg ='No message received in the last 5 sec from "%s"'%self._laptop_hostname
          self._laptop_battery_ctrl.set_stale()
          self._laptop_battery_ctrl.SetToolTip(wx.ToolTip(msg))
   
      if (rospy.is_shutdown()):
        self.Close()
        
    def on_rosout_clicked(self, evt):
        self._rosout_frame.Show()
        self._rosout_frame.Raise()

    def on_nav_clicked(self, evt):
        self._pub_goal.publish(True)

    def on_relais_clicked(self, evt):
        self._pub_relais.publish(not self._relais)

    def cb_battery_voltage(self, msg):
        wx.CallAfter(self.update_battery_voltage, msg)

    def update_battery_voltage(self, msg):
        self._last_battery_message_time = rospy.get_time()
        self._battery_state_ctrl.set_state(msg)

    def cb_relais(self, msg):
        wx.CallAfter(self.update_relais, msg)

    def voltage_to_perc(self, v):
        a = -7.87073944428413e-5
        b = -0.001363457642237
        c = 12.529846888629164

        if v > c: 
            return 100.0
    
        if v < 11.77:
            return 0.0

        return 100 + ( b + math.sqrt(b*b - 4*a*c + 4*a*v)) / (2 * a)


    def cb_sysinfo(self, msg):
        if msg.hostname == self._robot_hostname:
            # update robot status
            v = msg.battery_voltage
            self._last_robot_message = rospy.get_time()
            self._robot_battery_ctrl.set_voltage(v)
            self._robot_battery_ctrl.set_percent(self.voltage_to_perc(v))
            charging = False
            if v > 13.0:
                charging = True
            self._robot_battery_ctrl.set_charging(charging)

        elif msg.hostname == self._laptop_hostname:
            # update laptop status
            self._last_laptop_message = rospy.get_time()
            self._laptop_battery_ctrl.set_percent(msg.battery_percent)
            self._laptop_battery_ctrl.set_voltage(msg.battery_voltage)
            self._laptop_battery_ctrl.set_charging(msg.battery_plugged_in)

        else:
            rospy.logerr("received msg on %s from unknown host: %s."%(self._sub_sysinfo.topic(), msg.hostname))
        


    def update_relais(self, msg):
        self._last_relais_message = rospy.get_time()
        self._relais = msg.data
        if self._relais:
            self._relais_ctrl.set_ok()
            self._relais_ctrl.SetToolTip(wx.ToolTip("Relais: on"))
        else:
            self._relais_ctrl.set_error()
            self._relais_ctrl.SetToolTip(wx.ToolTip("Relais: off"))

    def cb_runstop(self, msg):
        wx.CallAfter(self.update_runstop, msg)

    def cb_runstop_wireless(self, msg):
        wx.CallAfter(self.update_runstop_wireless, msg)


    def update_runstop(self, msg):
        self._last_runstop_message = rospy.get_time()
        if not msg.data:
            self._runstop_ctrl.SetToolTip(wx.ToolTip("Runstop: on"))
            self._runstop_ctrl.set_ok()
        else:
            self._runstop_ctrl.SetToolTip(wx.ToolTip("Runstop: pressed"))
            self._runstop_ctrl.set_error()


    def cb_goal(self, msg):
        wx.CallAfter(self.update_goal, msg)

    def update_goal(self, msg):
        self._last_goal_message = rospy.get_time()
        if msg.data:
            self._goal_ctrl.SetToolTip(wx.ToolTip("Autonomous navigation: on"))
            self._goal_ctrl.set_ok()
        else:
            self._goal_ctrl.SetToolTip(wx.ToolTip("Autonomous navigation: off"))
            self._goal_ctrl.set_error()

    def update_runstop_wireless(self, msg):
        self._last_runstop_wireless_message = rospy.get_time()
        if not msg.data:
            self._runstop_wireless_ctrl.SetToolTip(wx.ToolTip("Wireless Runstop: on"))
            self._runstop_wireless_ctrl.set_ok()
        else:
            self._runstop_wireless_ctrl.SetToolTip(wx.ToolTip("Wireless Runstop: pressed"))
            self._runstop_wireless_ctrl.set_error()

          
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
      
