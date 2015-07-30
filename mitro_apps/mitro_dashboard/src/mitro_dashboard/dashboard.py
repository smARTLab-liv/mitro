import roslib;roslib.load_manifest('mitro_dashboard')
import rospy

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget
#from QtGui import QMessageBox, QAction
from PyQt4 import QtCore#, QtGui
#from python_qt_binding.QtCore import QSize

import std_msgs.msg
import mitro_diagnostics.msg
import geometry_msgs.msg
from std_srvs.srv import Empty

import math
import os

from .battery_widget import BatteryWidget
from .clear_costmap_widget import ClearCostmapWidget
from .navigation_widget import NavigationWidget
from .wifi_widget import WifiWidget
from .cpu_widget import CpuWidget
from .runstop_widget import RunstopWidget
from .relais_widget import RelaisWidget
from .home_widget import HomeWidget
from .assisted_drive_widget import AssistedDriveWidget


class MitroDashboard(Dashboard):

    #def __init__(self,context):
    #    self.name = "MITRO Dashboard"
    #    super(MitroDashboard, self).__init__(context)
    
    def setup(self, context):
        self._last_sysinfo_message = 0.0
        self._last_relais_message = 0.0
        self._last_runstop_message = 0.0
        self._last_pose_message = 0.0
        self._last_nav_message = 0.0
        self._last_assisted_drive_message = 0.0

        # Timer
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.on_timer)
        self._timer.start(500)

        # Runstop
        self._runstop_widget = RunstopWidget()

        # Relais
        self._relais_widget = RelaisWidget()
                                        
        # Battery
        self._battery_widget = BatteryWidget()

        # WiFi
        self._wifi_widget = WifiWidget()

        # CPU
        self._cpu_widget = CpuWidget()

        # Navigation
        self._nav_widget = NavigationWidget()

        # Clear costmap
        self._costmap_widget = ClearCostmapWidget()

        # Take me home
        self._home_widget = HomeWidget()

        # Assisted teleop
        self._assisted_drive_widget = AssistedDriveWidget()

        self._sub_runstop = rospy.Subscriber('runstop', std_msgs.msg.Bool, self.cb_runstop)
        self._sub_relais = rospy.Subscriber('relais', std_msgs.msg.Bool, self.cb_relais)
        self._sub_sysinfo = rospy.Subscriber('sysinfo', mitro_diagnostics.msg.SysInfo, self.cb_sysinfo)
        self._sub_nav = rospy.Subscriber("/goal_planner/has_goal", std_msgs.msg.Bool, self.cb_nav)
        self._sub_amcl_pose = rospy.Subscriber('/amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, self.cb_pose) 
        self._sub_assisted_drive = rospy.Subscriber("/assisted_drive/status", std_msgs.msg.Bool, self.cb_assisted_drive)


    def get_widgets(self):
	return [[MonitorDashWidget(self.context), ConsoleDashWidget(self.context)], 
		[self._runstop_widget, self._relais_widget], 
		[self._battery_widget, self._wifi_widget, self._cpu_widget],		
		[self._nav_widget, self._costmap_widget, self._assisted_drive_widget, self._home_widget] 
		]
		

    def shutdown_dashboard(self):
        self._sub_runstop.unregister()
        self._sub_relais.unregister()
        self._sub_sysinfo.unregister()
        self._sub_nav.unregister()
        self._sub_amcl_pose.unregister()
        self._sub_assisted_drive.unregister()

  
    def on_timer(self):
        if (rospy.get_time() - self._last_relais_message > 5.0):
            self._relais_widget.set_stale()

        if (rospy.get_time() - self._last_runstop_message > 5.0):
            self._runstop_widget.set_stale()

        #if (rospy.get_time() - self._last_pose_message > 5.0):
        #    self._home_widget.set_stale()

        if (rospy.get_time() - self._last_nav_message > 5.0):
            self._nav_widget.set_stale()

        if (rospy.get_time() - self._last_sysinfo_message > 5.0):
            self._cpu_widget.set_stale()
            self._battery_widget.set_stale()
            self._wifi_widget.set_stale()


    def cb_pose(self, msg):
        self._last_pose_message = rospy.get_time()
        self._home_widget.update_current_location(msg)
        
    def cb_relais(self, msg):
        self._last_relais_message = rospy.get_time()
        self._relais_widget.update(msg)

    def cb_sysinfo(self, msg):
        self._last_sysinfo_message = rospy.get_time()
        self._battery_widget.update(msg.battery_pc.percent, msg.battery_pc.voltage, msg.battery_pc.plugged_in)
        self._wifi_widget.update(msg.network.wifi_signallevel)
        self._cpu_widget.update(msg.system.cpu_usage_average, msg.system.cpu_temp_average)
        
    def cb_assisted_drive(self, msg):
        self._last_assisted_drive_message = rospy.get_time()
        self._assisted_drive_widget.update(msg)     
    
    def cb_runstop(self, msg):
        self._last_runstop_message = rospy.get_time()
        self._runstop_widget.update(msg)

    def cb_nav(self, msg):
        self._last_nav_message = rospy.get_time()
        self._nav_widget.update(msg)

        
    def voltage_to_perc(self, v):
        a = -7.87073944428413e-5
        b = -0.001363457642237
        c = 12.529846888629164

        if v > c: 
            return 100.0
    
        if v < 11.77:
            return 0.0

        return 100 + ( b + math.sqrt(b*b - 4*a*c + 4*a*v)) / (2 * a)
      
