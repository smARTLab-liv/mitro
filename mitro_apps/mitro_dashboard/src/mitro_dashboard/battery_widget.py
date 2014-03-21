import rospy
import roslib

from os import path
from rqt_robot_dashboard.widgets import IconToolButton
from rqt_robot_dashboard import util
from python_qt_binding.QtCore import QSize

class BatteryWidget(IconToolButton):

    def __init__(self):
        icons_path = path.join(roslib.packages.get_pkg_dir('mitro_dashboard'), "icons/")
        icons = []
        charge_icons = []
        icons.append([icons_path + 'battery-0.png'])
        icons.append([icons_path + 'battery-1.png'])
        icons.append([icons_path + 'battery-2.png'])
        icons.append([icons_path + 'battery-3.png'])
        icons.append([icons_path + 'battery-4.png'])
        icons.append([icons_path + 'battery-5.png'])
        icons.append([icons_path + 'battery-0-chrg.png'])
        icons.append([icons_path + 'battery-1-chrg.png'])
        icons.append([icons_path + 'battery-2-chrg.png'])
        icons.append([icons_path + 'battery-3-chrg.png'])
        icons.append([icons_path + 'battery-4-chrg.png'])
        icons.append([icons_path + 'battery-5-chrg.png'])
        icons.append([icons_path + 'battery-stale.png'])
        
        super(BatteryWidget, self).__init__('', icons=icons)
        
        self.setFixedSize(QSize(40,40))
        
        self.update_state(12)
        
        self._voltage = -1
        self._perc = -1
        self._charging = False


    def update(self, perc, volt, charging):
        self._charging = charging
        self._perc = round(perc)
        self._voltage = volt
        
        if not self._charging:       
            self.update_state(int(perc / 20.0))
        else:
            self.update_state(int(perc / 20.0) + 6)


    def update_state(self, state):
        super(BatteryWidget, self).update_state(state)
        if state is 12:
            self.setToolTip("Battery: Stale")
        else:
            self.setToolTip("Battery: %.0f %% (%.2f V)"%(self._perc, self._voltage))


    def set_stale(self):
        super(BatteryWidget, self).update_state(6)
