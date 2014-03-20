import rospy
import roslib
from functools import partial
from os import path
from rqt_robot_dashboard.widgets import MenuDashWidget
from rqt_robot_dashboard import util
from python_qt_binding.QtCore import QSize

from geometry_msgs.msg import PoseStamped

class HomeWidget(MenuDashWidget):

    def __init__(self):
        icons_path = path.join(roslib.packages.get_pkg_dir('mitro_dashboard'), "icons/")

        self._stale_icon = [icons_path + 'home-stale.png']
        self._online_icon = [icons_path + 'home-online.png']
        self._offline_icon = [icons_path + 'home-offline.png']

        icons = [self._stale_icon, self._online_icon, self._offline_icon]
        super(HomeWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40,40))

        self.add_action('Set home location', partial(self.set_home))
        self.add_action('Take me home!', partial(self.go_home))

        self._pub_set_goal = rospy.Publisher("/goal_planner/goal", PoseStamped) 

        self.update_state(0)
        self._last_pose = None
        self._home_goal = None

    def update_state(self, state):
        super(HomeWidget, self).update_state(state)
        if state is 0:
            self.setToolTip("Current position unknown")
        elif state is 1:
            self.setToolTip("Home location set")
        else:
            self.setToolTip("Home location not set")

    def close(self):
        self._pub_set_goal.unregister()

    def update_current_location(self, msg):
        self._last_pose = msg.pose.pose
        if self._home_goal is None:
            update_status(2)

    def set_stale(self):
	    self.update_state(0)
	    self._last_pose = None
	    self._home_goal = None

    def set_home(self):
        if self._last_pose is not None:
            self._home_goal = self._last_pose
            self.update_status(1)
            rospy.loginfo("New home location set")
        else:
            rospy.logwarn("Current position unknown, cannot set home location")
    
    def go_home(self):
        if self._home_goal is not None:
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "/map"
            msg.pose = self._home_goal
            self._pub_set_goal.publish(msg)
            rospy.loginfo("Going home")
        else:
            rospy.logwarn("No home location set")
                
