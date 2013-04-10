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
        f = open( "/sys/class/power_supply/" + battery_name + "/energy_full" )
        battery_max = float(f.readline())
        f.close()
    except:
        rospy.logerr("Cannot open battery state file: /sys/class/power_supply/" + battery_name + "/energy_full.")


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
    
    battery_percent = -1
    battery_time = -1
    battery_plugged_in = False
    battery_voltage = -1
    
    try:
        f = open( "/sys/class/power_supply/" + battery_name + "/power_now" )
        rate = float(f.readline())
        f.close()

        f = open( "/sys/class/power_supply/" + battery_name + "/energy_now" )
        cap = float(f.readline())
        battery_percent = (cap / battery_max) * 100.0
        battery_percent = max(0.0, battery_percent)
        battery_percent = min(100.0, battery_percent)
        battery_time = -1
        if rate > 0:
            battery_time = cap / rate        
        f.close()

        f = open( "/sys/class/power_supply/" + battery_name + "/status" )
        state = f.readline()
        if 'Charging' in state:
            battery_plugged_in = True
        f.close()

        f = open( "/sys/class/power_supply/" + battery_name + "/voltage_now" )
        battery_voltage = float(f.readline())
        f.close()
    except:
        rospy.logerr("Cannot open battery state file: /proc/acpi/battery/" + battery_name + "/state.")

            
def cb_bat_volt(msg):
    global battery_voltage
    battery_voltage = msg.data
    
def voltage_to_perc(v):
    a = -7.87073944428413e-5
    b = -0.001363457642237
    c = 12.529846888629164

    if v > c: 
        return 100.0

    if v < 11.77:
        return 0.0

    return 100 + ( b + math.sqrt(b*b - 4*a*c + 4*a*v)) / (2 * a)
    
def sysinfo():
    global battery_name
    
    rospy.init_node('sysinfo')
    pub = rospy.Publisher('sysinfo', SysInfo)
    
    wifi_name = 'wlan0'
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
        info_msg.system.cpu_usage_average = psutil.cpu_percent(interval=0.0)
        info_msg.system.cpu_usage_detail = psutil.cpu_percent(interval=0.0, percpu=True)
        info_msg.system.mem_usage = psutil.phymem_usage()[3]
        info_msg.network.wifi_signallevel = -1.0

        info_msg.network.ethernet_status = network_up("eth0") 

        try:
            info_msg.network.wifi_signallevel = wifi.getStatistics()[1].getSignallevel()
        except:
            pass

        if use_battery_voltage:
            info_msg.battery.voltage = battery_voltage
            info_msg.battery.plugged_in = False
            info_msg.battery.percent = voltage_to_perc(battery_voltage)
            info_msg.battery.time = -1
        else:
            battery_status()
            info_msg.battery.voltage = battery_voltage
            info_msg.battery.plugged_in = battery_plugged_in
            info_msg.battery.percent = battery_percent
            info_msg.battery.time = battery_time
        pub.publish(info_msg)
        r.sleep()
        
if __name__ == '__main__':
    try:
        sysinfo()
    except rospy.ROSInterruptException: pass
