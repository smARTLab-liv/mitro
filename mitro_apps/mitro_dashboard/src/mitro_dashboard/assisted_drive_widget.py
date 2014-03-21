import rospy
import roslib

from os import path
import actionlib
from rqt_robot_dashboard.widgets import IconToolButton
from python_qt_binding.QtCore import QSize
from std_msgs.msg import Bool

class AssistedDriveWidget(IconToolButton):
    def __init__(self):
        icons_path = path.join(roslib.packages.get_pkg_dir('mitro_dashboard'), "icons/")

        self._stale_icon = [icons_path + 'assisted-drive-stale.png']
        self._online_icon = [icons_path + 'assisted-drive-online.png']
        self._offline_icon = [icons_path + 'assisted-drive-offline.png']

        icons = [self._stale_icon, self._online_icon, self._offline_icon]
        super(AssistedDriveWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40,40))

        self.update_state(0)

        self.clicked.connect(self.toggle)
        self._assisted_drive_state = False
        self._pub_assisted_drive = rospy.Publisher("/assisted_drive/set", Bool)


    def update_state(self, state):
        super(AssistedDriveWidget, self).update_state(state)
        if state is 0:
            self.setToolTip("Assisted Drive: Stale")
        elif state is 1:
            self.setToolTip("Assisted Drive: Online")
        else:
            self.setToolTip("Assisted Drive: Offline")

    def update(self, msg):
        self._assisted_drive_state = msg.data
        if msg.data:
            self.update_state(1)
        else:
            self.update_state(2)

    def toggle(self):
        self._pub_assisted_drive.publish(not self._assisted_drive_state)
        rospy.loginfo("Assisted drive toggled")

    def close(self):
        self._pub_assisted_drive.unregister()

