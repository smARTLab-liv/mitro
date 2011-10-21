#!/usr/bin/env python

import math
import os
import curses
from threading import Timer
import time
from pythonwifi.iwlibs import Wireless
import psutil

import roslib; roslib.load_manifest("mitro_status")
import rospy
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry

import sys
 
wifi = None


def cpu_usage():
    return psutil.cpu_percent()

def uptime():
    try:
        f = open( "/proc/uptime" )
        contents = f.read().split()
        f.close()
    except:
        return "Cannot open uptime file: /proc/uptime"
    
    total_seconds = float(contents[0])
    
     # Helper vars:
    MINUTE  = 60
    HOUR    = MINUTE * 60
    DAY     = HOUR * 24
    
     # Get the days, hours, etc:
    days    = int( total_seconds / DAY )
    hours   = int( ( total_seconds % DAY ) / HOUR )
    minutes = int( ( total_seconds % HOUR ) / MINUTE )
    seconds = int( total_seconds % MINUTE )
 
     # Build up the pretty string (like this: "N days, N hours, N minutes, N seconds")
    string = ""
    if days > 0:
        string += str(days) + " " + "d" + ", "
    if len(string) > 0 or hours > 0:
        string += str(hours) + " " + "h" + ", "
    if len(string) > 0 or minutes > 0:
        string += str(minutes) + " " + "m" + ", "
    string += str(seconds) + " " + "s"
    return string;

def voltage_to_perc(v):
    a = -7.87073944428413e-5
    b = -0.001363457642237
    c = 12.529846888629164

    if v > c: 
        return "100%"
    
    if v < 11.77:
        return "low"

    return str(int(100 + ( b + math.sqrt(b*b - 4*a*c + 4*a*v)) / (2 * a))) + "%"

class MitroStatus():

    TITLE = "MITRO STATUS"
    CLOCK_UPDATE = 0.1

    WARN_VOLT = 11.77
    MIN_VOLT = 11.0

    WARN_WIFI = 70.0
    MIN_WIFI = 50.0

    WARN_CPU = 75.0

    FILTER_LENGTH = 20
    CPU_FILTER_LENGTH = 10

    lin_battery = 4
    lin_runstop = 6
    lin_relais = 7
    lin_speed = 8
    lin_position = 10
    lin_uptime = 16
    lin_rostime = 17
    lin_signallevel = 12
    lin_cpu = 14

    def signallevel(self): 
        global wifi
        return wifi.getStatistics()[1].getSignallevel()


    def cb_odom(self, msg):
        self.stdscr.addstr(self.lin_speed, 25, "%5.2f m/s"%msg.twist.twist.linear.x)
        self.stdscr.clrtoeol()
        
        self.stdscr.addstr(self.lin_speed + 1, 25, "%5.2f rad/s"%msg.twist.twist.angular.z)
        self.stdscr.clrtoeol()
        
        o = msg.pose.pose.orientation
        # TODO: doesn't seem to work
        yaw = math.atan2(2.0*(o.x*o.w + o.y* o.z), 1.0 - 2.0*(o.z*o.z + o.w*o.w))

        self.stdscr.addstr(self.lin_position, 25, "%6.2f m, %6.2f m, %6.3f rad"%(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))
        self.stdscr.clrtoeol()
                
        self.stdscr.refresh()

    def cb_battery_voltage(self, msg):
        color = self.TEXT_OK
        if msg.data < self.WARN_VOLT:
            color = self.TEXT_BATTERYWARN
        if msg.data < self.MIN_VOLT:
            color = self.TEXT_ERROR
        s = " " * max((int(msg.data * 3) - 8), 0) 
        self.stdscr.addstr(self.lin_battery, 25, (" %5.2f V (%s)" + s)%(msg.data, voltage_to_perc(msg.data)), color)
        self.stdscr.clrtoeol()
        self.stdscr.refresh()    

    def cb_runstop(self, msg):
        if msg.data:
            self.stdscr.addstr(self.lin_runstop, 25, "    STOP    ", self.TEXT_ERROR)
            self.stdscr.clrtoeol()
        else:
            self.stdscr.addstr(self.lin_runstop, 25, "     GO     ", self.TEXT_OK)
            self.stdscr.clrtoeol()
        self.stdscr.refresh()    

    def cb_relais(self, msg):
        if msg.data:
            self.stdscr.addstr(self.lin_relais, 25, "     GO     ", self.TEXT_OK)
            self.stdscr.clrtoeol()
        else:
            self.stdscr.addstr(self.lin_relais, 25, "    STOP    ", self.TEXT_ERROR)
            self.stdscr.clrtoeol()
        self.stdscr.refresh()    

        
    def update_clock(self):
        self.stdscr.addstr(self.lin_rostime, 25, "%13.2f"%(rospy.Time.now().to_sec()))
        self.stdscr.clrtoeol()
        self.stdscr.addstr(self.lin_uptime, 25, uptime());
        self.stdscr.clrtoeol()
        
        self.signal_list.append(-self.signallevel())
        if len(self.signal_list) > self.FILTER_LENGTH:
            self.signal_list = self.signal_list[1:]
        level = sum(self.signal_list)/len(self.signal_list)

        if level < 30:
            level = 30
        if level > 95:
            level = 95
        perc = 100 - (level-30) * 100.0/65.0
        s = " " * (int(0.4 * perc) - 14)
        color = self.TEXT_OK
        if perc < self.WARN_WIFI:
            color = self.TEXT_BATTERYWARN
        if perc < self.MIN_WIFI:
            color = self.TEXT_ERROR
        self.stdscr.addstr(self.lin_signallevel, 25, (" %3d dBm (%3d%%)" + s)%(self.signallevel(), perc), color)
        self.stdscr.clrtoeol()

        self.cpu_list.append(cpu_usage())
        if len(self.cpu_list) > self.CPU_FILTER_LENGTH:
            self.cpu_list = self.cpu_list[1:]
        cpu_perc = sum(self.cpu_list)/len(self.cpu_list)

        #cpu_perc = cpu_usage()
        s = " " * (int(0.4 * cpu_perc) - 9)
        color = self.TEXT_OK
        if cpu_perc > self.WARN_CPU:
            color = self.TEXT_BATTERYWARN
	str_perc = str('%3.1f'%cpu_perc)
        self.stdscr.addstr(self.lin_cpu, 25, (" %s %%" + s)%str_perc.rjust(5), color)
        self.stdscr.clrtoeol()


        self.stdscr.refresh()
        #self.counter += 1
        #self.clock_timer = Timer(self.CLOCK_UPDATE, self.update_clock)
        #self.clock_timer.start()
    
    def init(self):
        # get window dimensions
        (self.max_x, self.max_y) = self.stdscr.getmaxyx()
        
        # curser of
        try:
            curses.curs_set(0)
        except Exception, e:
            pass
        
        # setup colors
        curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_RED)
        curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_GREEN)
        curses.init_pair(4, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(5, curses.COLOR_BLACK, curses.COLOR_YELLOW)
        self.TEXT_NORMAL = curses.color_pair(1)
        self.TEXT_ERROR = curses.color_pair(2)
        self.TEXT_OK = curses.color_pair(3)
        self.TEXT_WARNING = curses.color_pair(4)
        self.TEXT_BATTERYWARN = curses.color_pair(5)

        #self.stdscr.border(0)
        self.stdscr.addstr(1, 1, self.TITLE, self.TEXT_NORMAL)
        
        self.stdscr.addstr(self.lin_battery, 1, "Battery Voltage: ", self.TEXT_NORMAL)
        self.stdscr.addstr(self.lin_battery, 25, "not published yet", self.TEXT_WARNING)

        self.stdscr.addstr(self.lin_runstop, 1, "Runstop: ", self.TEXT_NORMAL)
        self.stdscr.addstr(self.lin_runstop, 25, "not published yet", self.TEXT_WARNING)

        self.stdscr.addstr(self.lin_relais, 1, "Relais: ", self.TEXT_NORMAL)
        self.stdscr.addstr(self.lin_relais, 25, "not published yet", self.TEXT_WARNING)

        self.stdscr.addstr(self.lin_speed, 1, "Linear speed: ", self.TEXT_NORMAL)
        self.stdscr.addstr(self.lin_speed, 25, "not published yet", self.TEXT_WARNING)

        self.stdscr.addstr(self.lin_speed + 1, 1, "Angular speed: ", self.TEXT_NORMAL)
        self.stdscr.addstr(self.lin_speed + 1, 25, "not published yet", self.TEXT_WARNING)

        self.stdscr.addstr(self.lin_position, 1, "Position: ", self.TEXT_NORMAL)
        self.stdscr.addstr(self.lin_position, 25, "not published yet", self.TEXT_WARNING)

        self.stdscr.addstr(self.lin_uptime, 1, "Uptime: ", self.TEXT_NORMAL)
        self.stdscr.addstr(self.lin_rostime, 1, "ROS time:", self.TEXT_NORMAL)
        self.stdscr.addstr(self.lin_signallevel, 1, "WiFi signal level:", self.TEXT_NORMAL)
        self.stdscr.addstr(self.lin_cpu, 1, "CPU usage:", self.TEXT_NORMAL)

        #self.stdscr.addstr(2, 1, " Error ", curses.color_pair(2) )
        #self.stdscr.addstr(3, 1, " Ok ", curses.color_pair(3) )
        #self.stdscr.addstr(4, 1, " Warning ", curses.color_pair(4) )

        hostname = os.uname()[1]
        self.stdscr.addstr(1, self.max_y - len(hostname) - 1, hostname, self.TEXT_NORMAL)

        self.stdscr.refresh()

    def main(self, stdscr):
        self.stdscr = stdscr
        self.init()
	
        self.signal_list = []
	self.cpu_list = []
        self.counter = 0
        #self.update_clock()

        while not rospy.is_shutdown():
            self.update_clock()
            rospy.sleep(.1)    

        # cancel clock and join thread
        #self.clock_timer.cancel()
        #self.clock_timer.join() #we tried to be nice but it doesn't work
	#sys.exit(0)
	

if __name__ == "__main__":
    nh = rospy.init_node("mitro_status");
    wifiName=""
    try:
        wifiName = rospy.get_param("wifi")
        wifi = Wireless(wifiName)
    except KeyError:
        rospy.logerr("wifi name not set")
        sys.exit(1)
    if not rospy.is_shutdown():
    	mc = MitroStatus()
    	mc.sub_battery = rospy.Subscriber("battery_voltage", Float32, mc.cb_battery_voltage)
    	mc.sub_runstop = rospy.Subscriber("runstop", Bool, mc.cb_runstop)
	mc.sub_relais = rospy.Subscriber("relais", Bool, mc.cb_relais)
    	mc.sub_odom = rospy.Subscriber("odom", Odometry, mc.cb_odom)
    	curses.wrapper(mc.main)


