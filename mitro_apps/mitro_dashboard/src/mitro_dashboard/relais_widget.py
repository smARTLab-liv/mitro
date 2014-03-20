import rospy
import roslib

from os import path
from rqt_robot_dashboard.widgets import IconToolButton
from rqt_robot_dashboard import util
from python_qt_binding.QtCore import QSize

from std_msgs.msg import Bool

class RelaisWidget(IconToolButton):

    def __init__(self):
        icons_path = path.join(roslib.packages.get_pkg_dir('mitro_dashboard'), "icons/")

        self._off_icon = [icons_path + 'motor-stale.png']
        self._green_icon = [icons_path + 'motor-online.png']
        self._red_icon = [icons_path + 'motor-offline.png']

        icons = [self._off_icon, self._green_icon, self._red_icon]
        super(RelaisWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40,40))

        self.clicked.connect(self.switch_relais)

        self.update_state(0)
        self._relais_state = None

        self._pub_relais = rospy.Publisher('cmd_relais', Bool)

    def update_state(self, state):
        super(RelaisWidget, self).update_state(state)
        if state is 0:
            self.setToolTip("Relais: Stale")
        elif state is 1:
            self.setToolTip("Relais: Switched ON")
        elif state is 2:
            self.setToolTip("Relais: Switched OFF")
        else:
            self.setToolTip("Relais: ERROR")
    
    def update(self, msg):
        self._relais_state = msg.data
        if (msg.data == True):
            self.update_state(1)
        else:
            self.update_state(2)

    def close(self):
        self._pub_relais.unregister()

    def switch_relais(self):
        self._pub_relais.publish(not self._relais_state)
    
    def set_stale(self):
        self.update_state(0)
        
