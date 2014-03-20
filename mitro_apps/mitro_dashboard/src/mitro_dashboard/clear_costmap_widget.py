import rospy
import roslib

from os import path
import actionlib
from rqt_robot_dashboard.widgets import IconToolButton
from python_qt_binding.QtCore import QSize
from std_srvs.srv import Empty

class ClearCostmapWidget(IconToolButton):
    def __init__(self):
        icons_path = path.join(roslib.packages.get_pkg_dir('mitro_dashboard'), "icons/")

        self._stale_icon = [icons_path + 'costmap-stale.png']
        self._online_icon = [icons_path + 'costmap-online.png']


        icons = [self._stale_icon, self._online_icon]
        super(ClearCostmapWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40,40))
        self.update_state(0)
        
        self.clicked.connect(self.toggle)
        
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', 1)
            self._srv_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            self.update_state(1)
            self._costmap_ok = True
        except rospy.ROSException, e:
            rospy.logwarn("move base not running yet, can't clear costmap")
            self.update_state(0)
            self._costmap_ok = False

    def update_state(self, state):
        super(ClearCostmapWidget, self).update_state(state)
        if state is 0:
            self.setToolTip("Move base not running")
        else:
            self.setToolTip("Clear Costmap")

    def toggle(self):
        if not self._costmap_ok: 
            self._srv_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
        try:            
            self._srv_costmap()
            self.update_state(1)
            self._costmap_ok = True
            rospy.loginfo("Costmap cleared")
        except rospy.ServiceException, e:
            rospy.logwarn("move base not running yet, can't clear costmap")
            self.update_state(0)
            self._costmap_ok = False

    def close(self):
        try:
            self._srv_costmap.unregister()
        except:
            rospy.logerr("Failed to unregister clear costmap service")
