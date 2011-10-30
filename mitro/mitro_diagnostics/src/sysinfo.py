#!/usr/bin/env python
import roslib; roslib.load_manifest("mitro_diagnostics")
import rospy
from mitro_diagnostics.msg import SysInfo
from std_msgs.msg import Float32
from pythonwifi.iwlibs import Wireless
import psutil
import socket
import math

battery_name = ''
battery_max = -1
battery_percent = -1
battery_time = -1
battery_voltage = -1
battery_plugged_in = False

def battery_max():
    global battery_name, battery_max
    try:
        f = open( "/proc/acpi/battery/" + battery_name + "/info" )
        contents = f.read().split('\n')
        f.close()
    except:
        print "Cannot open battery state file: /proc/acpi/battery/" + battery_name + "/state"
    
    for line in contents:
        if 'last full capacity' in line:
            el = line.split()
            battery_max = float(el[3])

def network_up(name):
    fn = "/sys/class/net/%s/operstate"%name
    try:
        f = open( fn )
        state = f.read().strip()
        return ( state == "up" )
    except:
        rospy.logerr("Can't open file %s"%fn)
        return False



def battery_status():
    global battery_name, battery_max, battery_percent, battery_time, battery_plugged_in, battery_voltage
    try:
        f = open( "/proc/acpi/battery/" + battery_name + "/state" )
        contents = f.read().split('\n')
        f.close()
    except:
        rospy.logerr("Cannot open battery state file: /proc/acpi/battery/$s/state."%battery_name)

    battery_percent = -1
    battery_time = -1
    battery_plugged_in = False
    battery_voltage = -1

    rate = 0
    for line in contents:
        if 'present rate' in line:
            rate = float(line.split()[2])
        if 'remaining capacity' in line:
            cap = float(line.split()[2])
            battery_percent = (cap / battery_max) * 100.0
            battery_percent = max(0.0, battery_percent)
            battery_percent = min(100.0, battery_percent)
            battery_time = -1
            if rate > 0:
                battery_time = cap / rate
        if 'charging state' in line:
            state = line.split()[2]
            if not state == 'discharging':
                battery_plugged_in = True
        if 'present voltage' in line:
            battery_voltage = float(line.split()[2]) / 1000.0
            
def cb_bat_volt(msg):
    global battery_voltage
    battery_voltage = msg.data
    
def sysinfo():
    global battery_name
    
    rospy.init_node('sysinfo')
    pub = rospy.Publisher('sysinfo', SysInfo)
    
    wifi_name = 'wlan1'
    if rospy.has_param('~wifi_name'):
        wifi_name = rospy.get_param('~wifi_name')
    wifi = Wireless(wifi_name)
    
    use_battery_voltage = True
    battery_name = 'BAT0'
    if rospy.has_param('~battery_name'):
        battery_name = rospy.get_param('~battery_name')
        battery_max()
        use_battery_voltage = False

    if use_battery_voltage:
        rospy.Subscriber('battery_voltage', Float32, cb_bat_volt)

    
    info_msg = SysInfo()
    info_msg.hostname = socket.gethostname()
    
    r = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        info_msg.header.stamp = rospy.Time.now()
        info_msg.cpu_usage = psutil.cpu_percent(interval=0.0)
        info_msg.cpu_usage_detail = psutil.cpu_percent(interval=0.0, percpu=True)
        info_msg.mem_usage = psutil.phymem_usage()[3]
        info_msg.wifi_signallevel = -1.0

        info_msg.network_state = network_up("eth0") 

        try:
            info_msg.wifi_signallevel = wifi.getStatistics()[1].getSignallevel()
        except:
            pass

        if use_battery_voltage:
            info_msg.battery_voltage = battery_voltage
            info_msg.battery_plugged_in = False
            info_msg.battery_percent = -1
            info_msg.battery_time = -1
        else:
            battery_status()
            info_msg.battery_voltage = battery_voltage
            info_msg.battery_plugged_in = battery_plugged_in
            info_msg.battery_percent = battery_percent
            info_msg.battery_time = battery_time
        pub.publish(info_msg)
        r.sleep()
        
if __name__ == '__main__':
    try:
        sysinfo()
    except rospy.ROSInterruptException: pass
