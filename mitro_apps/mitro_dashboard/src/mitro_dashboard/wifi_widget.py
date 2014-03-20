import rospy
import roslib
from os import path
from rqt_robot_dashboard.widgets import IconToolButton
from rqt_robot_dashboard import util
from python_qt_binding.QtCore import QSize

class WifiWidget(IconToolButton):

    def __init__(self):
        icons_path = path.join(roslib.packages.get_pkg_dir('mitro_dashboard'), "icons/")

        icons = []
        for i in range(5):
            icons.append([icons_path + 'wifi-%d.png'%i])    

        icons.append([icons_path + 'wifi-stale.png'])

        super(WifiWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40,40))

        self.setToolTip("Wifi: Stale")

    def update_state(self, state):
        super(WifiWidget, self).update_state(state)
        if state is 5:  
            self._stall = True
            self.setToolTip("Wifi: Stale")

    def close(self):
        pass

    def set_stale(self):
	    self.update_state(5)

    def update(self, signallevel):
        self.setToolTip("Wifi signal level: %.1f dBm"%signallevel)

        if self._signallevel == -1:
            self.update_state(5)
        else:
            level = - self._signallevel
            if level < 30:
                level = 30
            if level > 95:
                level = 95
            perc = 100 - (level-30) * 100.0/65.0
            idx = int( (perc + 12.5) / 25.0 )
	    self.update_state(idx)
