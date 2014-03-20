import rospy
import roslib

from os import path
import actionlib
from rqt_robot_dashboard.widgets import IconToolButton
from python_qt_binding.QtCore import QSize
from std_msgs.msg import Bool

class NavigationWidget(IconToolButton):
    def __init__(self):
        icons_path = path.join(roslib.packages.get_pkg_dir('mitro_dashboard'), "icons/")

        self._stale_icon = [icons_path + 'nav-stale.png']
        self._online_icon = [icons_path + 'nav-online.png']
        self._offline_icon = [icons_path + 'nav-offline.png']

        icons = [self._stale_icon, self._online_icon, self._offline_icon]
        super(NavigationWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40,40))

        self.update_state(0)

        self.clicked.connect(self.cancel_goals)

        self._pub_goal = rospy.Publisher("/goal_planner/cancel", Bool)

    def update_state(self, state):
        super(NavigationWidget, self).update_state(state)
        if state is 0:
            self.setToolTip("Navigation: Stale")
        elif state is 1:
            self.setToolTip("Navigation in progress: Cancel?")
        else:
            self.setToolTip("Navigation offline")

    def update(self, msg):
        if msg.data:
            self.update_state(1)
        else:
            self.update_state(2)

    def cancel_goals(self):
        self._pub_goal.publish(True)
        rospy.loginfo("All goals cancelled")
        
    def set_stale(self):
        self.update_state(0)

    def close(self):
        self._pub_goal.unregister()

