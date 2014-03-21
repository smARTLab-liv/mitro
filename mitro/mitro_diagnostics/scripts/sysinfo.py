#!/usr/bin/env python
import rospy
from mitro_diagnostics.msg import SysInfo, SystemStatus, NetworkStatus, BatteryStatus
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from pythonwifi.iwlibs import Wireless
import psutil
import socket
import numpy
import os
import re

class SystemInfo():
    
    BAT_PERC_WARN = 40
    BAT_PERC_ERROR = 20
    BAT_VOLT_WARN = 11.5
    BAT_VOLT_ERROR = 11.0
    CPU_TEMP_WARN = 70
    CPU_TEMP_ERROR = 80
    CPU_USAGE_WARN = 90
    CPU_USAGE_ERROR = 95
    
    def __init__(self):
        rospy.init_node('sysinfo')
        pub = rospy.Publisher('sysinfo', SysInfo)
        pub_diagnostics = rospy.Publisher('/diagnostics', DiagnosticArray)
        rospy.Subscriber('battery_voltage', Float32, self.cb_bat_volt)
        
        wifi_name = 'wlan0'
        if rospy.has_param('~wifi_name'):
            wifi_name = rospy.get_param('~wifi_name')
        self._wifi = Wireless(wifi_name)

        self._eth_name = 'eth0'
        if rospy.has_param('~eth_name'):
            wifi_name = rospy.get_param('~eth_name')

        self._base_bat_voltage = -1

        self._stat_bat = []
        self._stat_network = []
        self._stat_system = []

        info_msg = SysInfo()
        info_msg.hostname = socket.gethostname()
        
        diag_msg = DiagnosticArray()
        
        r = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            info_msg.header.stamp = rospy.Time.now()
            info_msg.system = self.system_status()
            info_msg.network = self.network_status()
            info_msg.battery = self.battery_status()
            pub.publish(info_msg)
            
            diag_msg.header.stamp = rospy.Time.now()
            diag_msg.status.append(self.stat_bat)
            diag_msg.status.append(self.stat_network)
            diag_msg.status.append(self.stat_system)
            pub_diagnostics.publish(diag_msg)
            
            r.sleep()


    def network_status(self):
        msg = NetworkStatus()
        msg.wifi_signallevel = -1.0
        try:
            msg.wifi_signallevel = self._wifi.getStatistics()[1].getSignallevel()
        except:
            pass
        
        #fn = "/sys/class/net/%s/operstate"%(self._eth_name)
        #try:
        #    f = open( fn )
        #    state = f.read().strip()
        #    msg.ethernet_connected = True
        #except:
        #    rospy.logerr("Can't open file %s"%fn)
        #    msg.ethernet_connected = False
        msg.ethernet_connected = False        

        self.stat_network = DiagnosticStatus(name="computer: Network",level=DiagnosticStatus.OK,message="OK")
        self.stat_network.values = [KeyValue("WiFi signal strength (db)",str(msg.wifi_signallevel)),
                                    KeyValue("Ethernet connected",str(msg.ethernet_connected))]
        
        return msg


    def system_status(self):
        msg = SystemStatus()
        msg.cpu_usage_average = psutil.cpu_percent(interval=0.0)
        msg.cpu_usage_detail = psutil.cpu_percent(interval=0.0, percpu=True)
        msg.mem_usage = psutil.phymem_usage()[3]
        
        temps = []
        res = os.popen("sensors | grep Core")
        for line in res.readlines():
            temps.append(float(re.search('\+(.*?)\W\WC', line).group(1)))
        msg.cpu_temp_detail = temps
        msg.cpu_temp_average = numpy.mean(temps)
        
        self.stat_system = DiagnosticStatus(name="computer: System",level=DiagnosticStatus.OK,message="OK")
        self.stat_system.values = [ KeyValue("CPU usage",str(msg.cpu_usage_average)),
                                    KeyValue("CPU temp (C)",str(msg.cpu_temp_average)),
                                    KeyValue("Memory usage",str(msg.mem_usage))]
        
        if msg.cpu_temp_average > SystemInfo.CPU_TEMP_ERROR:
            self.stat_system.level = DiagnosticStatus.ERROR
            self.stat_system.message = "CPU overheating"
        elif msg.cpu_temp_average > SystemInfo.CPU_TEMP_WARN:
            self.stat_system.level = DiagnosticStatus.WARNING
            self.stat_system.message = "CPU overheating"    
        elif msg.cpu_usage_average > SystemInfo.CPU_USAGE_ERROR:
            self.stat_system.level = DiagnosticStatus.ERROR
            self.stat_system.message = "High CPU load"
        elif msg.cpu_usage_average > SystemInfo.CPU_USAGE_WARN:
            self.stat_system.level = DiagnosticStatus.WARN
            self.stat_system.message = "High CPU load"
                
        return msg   


    def battery_status(self):
        msg = BatteryStatus()
        
        msg.percent = self.voltage_to_perc(self._base_bat_voltage)
        if self._base_bat_voltage > 13.0:
            msg.plugged_in = True
        else:
            msg.plugged_in = False
        msg.voltage = self._base_bat_voltage
        msg.watt = -1
        msg.temp = -1
        
        self.stat_bat = DiagnosticStatus(name="battery: Base",level=DiagnosticStatus.OK,message="OK")
        self.stat_bat.values = [KeyValue("Voltage (V)",str(msg.voltage)),
                                KeyValue("Percentage",str(msg.percent)),
                                KeyValue("Power (W)",str(msg.watt)),
                                KeyValue("Temperature (C)",str(msg.temp)),
                                KeyValue("Charging",str(msg.plugged_in))]
        
        if msg.voltage < SystemInfo.BAT_VOLT_ERROR:
            self.stat_bat.level = DiagnosticStatus.ERROR
            self.stat_bat.message = "Battery almost empty"
        elif msg.voltage < SystemInfo.BAT_VOLT_WARN:
            self.stat_bat.level = DiagnosticStatus.WARN
            self.stat_bat.message = "Battery almost empty"
        
        return msg

    def cb_bat_volt(self, msg):
        self._base_bat_voltage = msg.data


    def voltage_to_perc(self, v):
        a = -7.87073944428413e-5
        b = -0.001363457642237
        c = 12.529846888629164
        
        if v > c: 
            return 100.0
        
        if v < 11.77:
            return 0.0
        
        return 100 + ( b + numpy.sqrt(b*b - 4*a*c + 4*a*v)) / (2 * a)


if __name__ == '__main__':
    try:
        obj = SystemInfo()
    except rospy.ROSInterruptException:
        pass
