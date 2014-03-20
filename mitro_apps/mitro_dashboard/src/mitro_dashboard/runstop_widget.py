import rospy
import roslib

from os import path
from rqt_robot_dashboard.widgets import IconToolButton
from rqt_robot_dashboard import util
from python_qt_binding.QtCore import QSize


class RunstopWidget(IconToolButton):

    def __init__(self):
        icons_path = path.join(roslib.packages.get_pkg_dir('mitro_dashboard'), "icons/")

        self._off_icon = [icons_path + 'runstop-stale.png']
        self._green_icon = [icons_path + 'runstop-online.png']
        self._red_icon = [icons_path + 'runstop-offline.png']

        icons = [self._off_icon, self._green_icon, self._red_icon]
        super(RunstopWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40,40))

        self.update_state(0)

    def update_state(self, state):
        super(RunstopWidget, self).update_state(state)
        if state is 0:
            self.setToolTip("Runstop: Stale")
        elif state is 1:
            self.setToolTip("Runstop: Switched ON")
        elif state is 2:
            self.setToolTip("Runstop: Switched OFF")
        else:
            self.setToolTip("Runstop: ERROR")

    def close(self):
        pass

    def update(self, msg):
        if msg.data:
            self.update_state(1)
        else:
            self.update_state(2)

    def set_stale(self):
        self.update_state(0)

