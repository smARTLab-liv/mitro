#!/usr/bin/env python
import rospy
from mitro_diagnostics.msg import SysInfo, SystemStatus, NetworkStatus, BatteryStatus
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from roboclaw_driver.msg import RoboClawState
import psutil
from nut2 import PyNUTClient
import socket
import numpy
import os
#import re
import sensors

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
        pub = rospy.Publisher('sysinfo', SysInfo, queue_size=1)
        pub_diagnostics = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        rospy.Subscriber('roboclaw_state', RoboClawState, self.cb_bat_volt)
        
        self._wifi_name = 'wlan0'
        if rospy.has_param('~wifi_name'):
            self._wifi_name = rospy.get_param('~wifi_name')

        self._eth_name = 'eth0'
        if rospy.has_param('~eth_name'):
            self._eth_name = rospy.get_param('~eth_name')

        self._base_bat_voltage = -1
        self._last_bat_base = rospy.Time.now()

        self._stat_bat_base = []
        self._stat_bat_pc = []
        self._stat_network = []
        self._stat_system = []

        info_msg = SysInfo()
        info_msg.hostname = socket.gethostname()
        
        sensors.init()
        
        diag_msg = DiagnosticArray()
        
        r = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            info_msg.header.stamp = rospy.Time.now()
            info_msg.system = self.system_status()
            info_msg.network = self.network_status()
            info_msg.battery_base = self.battery_base_status()
            info_msg.battery_pc = self.battery_pc_status()
            pub.publish(info_msg)
            
            diag_msg.header.stamp = rospy.Time.now()
            diag_msg.status.append(self.stat_bat_base)
            diag_msg.status.append(self.stat_bat_pc)
            diag_msg.status.append(self.stat_network)
            diag_msg.status.append(self.stat_system)
            pub_diagnostics.publish(diag_msg)
            
            r.sleep()


    def network_status(self):
        msg = NetworkStatus()
        msg.wifi_signallevel = -1.0
        msg.wifi_link_quality = "-1"
        try:
            f = os.popen('iwconfig %s | grep "Link Quality"' %(self._wifi_name))
            fs = f.read().split()
            msg.wifi_link_quality = fs[1].split('=')[1]
            msg.wifi_signallevel = int(fs[3].split('=')[1])
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
        
        # since we're never connected annyway. I mean, no way you're squeezing a cable in there..
        msg.ethernet_connected = False 

        self.stat_network = DiagnosticStatus(name="computer: Network",level=DiagnosticStatus.OK,message="OK")
        self.stat_network.values = [KeyValue("WiFi signal strength (db)",str(msg.wifi_signallevel)),
                                    KeyValue("WiFi link quality",msg.wifi_link_quality),
                                    KeyValue("Ethernet connected",str(msg.ethernet_connected))]
        
        return msg


    def system_status(self):
        msg = SystemStatus()
        msg.cpu_usage_average = psutil.cpu_percent(interval=0.0)
        msg.cpu_usage_detail = psutil.cpu_percent(interval=0.0, percpu=True)
        msg.mem_usage = psutil.phymem_usage()[3]
        
        temps = []
        #res = os.popen("sensors | grep Core")
        #for line in res.readlines():
        #    temps.append(float(re.search('\+(.*?)\W\WC', line).group(1)))
        try:
            for chip in sensors.iter_detected_chips():
                for feature in chip:
                    if "Core" in feature.label:
                        temps.append(feature.get_value())
        except:
            pass
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


    def battery_base_status(self):
        msg = BatteryStatus()

        # check if any cirrent messages were received
        if (rospy.Time.now() - self._last_bat_base) > rospy.Duration(5):
            self._base_bat_voltage = -1
        
        # this should be recalibrated 
        # msg.percent = self.voltage_to_perc(self._base_bat_voltage)
        msg.percent = -1;
        if self._base_bat_voltage > 13.0:
            msg.plugged_in = True
        else:
            msg.plugged_in = False
        msg.voltage = self._base_bat_voltage
        msg.current = -1
        msg.temp = -1
        
        self.stat_bat_base = DiagnosticStatus(name="battery: Base",level=DiagnosticStatus.OK,message="OK")
        self.stat_bat_base.values = [KeyValue("Voltage (V)",str(msg.voltage)),
                                KeyValue("Percentage",str(msg.percent)),
                                KeyValue("Current (A)",str(msg.current)),
                                KeyValue("Temperature (C)",str(msg.temp)),
                                KeyValue("Charging",str(msg.plugged_in))]

        if msg.voltage == -1:
            self.stat_bat_base.level = DiagnosticStatus.ERROR
            self.stat_bat_base.message = "No data received in the past 5 seconds"
        elif msg.voltage < SystemInfo.BAT_VOLT_ERROR:
            self.stat_bat_base.level = DiagnosticStatus.ERROR
            self.stat_bat_base.message = "Battery almost empty"
        elif msg.voltage < SystemInfo.BAT_VOLT_WARN:
            self.stat_bat_base.level = DiagnosticStatus.WARN
            self.stat_bat_base.message = "Battery almost empty"
        
        return msg


    def battery_pc_status(self):
        msg = BatteryStatus()
        
        msg.percent = -1
        msg.plugged_in = False
        msg.voltage = -1
        msg.current = -1
        msg.temp = -1

        success = False
        
        try:
            ups = PyNUTClient()
            bat = ups.list_vars("OpenUPS")
            msg.percent = float(bat['battery.charge'])
            msg.plugged_in = (str(bat['ups.status']) != "OB DISCHRG")
            msg.voltage = float(bat['battery.voltage'])
            if msg.plugged_in:
                msg.current = float(bat['input.current'])
            else:
                msg.current = -1 * float(bat['output.current']) * (float(bat['output.voltage']) / float(bat['battery.voltage']))
            msg.temp = float(bat['battery.temperature'])
            success = True
        except:
            rospy.logerr("Cannot connect to power board.")

        self.stat_bat_pc = DiagnosticStatus(name="battery: PC",level=DiagnosticStatus.OK,message="OK")
        self.stat_bat_pc.values = [KeyValue("Voltage (V)",str(msg.voltage)),
                                KeyValue("Percentage",str(msg.percent)),
                                KeyValue("Current (A)",str(msg.current)),
                                KeyValue("Temperature (C)",str(msg.temp)),
                                KeyValue("Charging",str(msg.plugged_in))]
        
        if not success:
            self.stat_bat_base.level = DiagnosticStatus.ERROR
            self.stat_bat_base.message = "Cannot connect to the power board"
        elif msg.percent < SystemInfo.BAT_PERC_ERROR:
            self.stat_bat_pc.level = DiagnosticStatus.ERROR
            self.stat_bat_pc.message = "Battery almost empty"
        elif msg.percent < SystemInfo.BAT_PERC_WARN:
            self.stat_bat_pc.level = DiagnosticStatus.WARN
            self.stat_bat_pc.message = "Battery almost empty"
        
        return msg


    def cb_bat_volt(self, msg):
        self._base_bat_voltage = msg.battery_voltage
        self._last_bat_base = rospy.Time.now()


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
