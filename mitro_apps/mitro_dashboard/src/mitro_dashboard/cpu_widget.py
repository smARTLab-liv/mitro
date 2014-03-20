import rospy
import roslib

from os import path
from rqt_robot_dashboard.widgets import IconToolButton
from rqt_robot_dashboard import util
from python_qt_binding.QtCore import QSize
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import QApplication, QLabel

from std_srvs.srv import Empty

class CpuWidget(IconToolButton, QtGui.QWidget):

    def __init__(self):
        icons_path = path.join(roslib.packages.get_pkg_dir('mitro_dashboard'), "icons/")   

        self._cpu_stale_icon = [icons_path + 'cpu-stale.png']
        self._cpu_bg_icon = [icons_path + 'cpu-bg.png']

        icons = [self._cpu_stale_icon, self._cpu_bg_icon]

        super(CpuWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40,40))       
        self._state = 0
        self.update_state(0)

    def update_state(self, state):
        super(CpuWidget, self).update_state(state)
        self._state = state
        if state is 0:
            self._cpu_usage_hist = {} #[10,20,15,30,50,50,50,80,100,10]
            self._cpu_usage = -1
            self.setToolTip("CPU: stale")

    def close(self):
        pass

    def set_stale(self):
        self.update_state(0)

    def update(self, cpu_usage, cpu_temp):
        self._cpu_usage_hist.insert(0, self._cpu_usage)
        if len(self._cpu_usage_hist) > 10:
            self._cpu_usage_hist.pop()
        tooltip = "CPU: %.0f%%, %.0fC"%(cpu_usage, cpu_temp)
        self.setToolTip(tooltip)
        self.show()
        self.update_state(1)

    def draw(self, qp):
        # a whole bunch of random number here.. but they seem to work :)
        target = QtCore.QRectF(5, 5, 41, 41)
        source = QtCore.QRectF(0, 0, 48, 48)
        if self._state is 0:
            qp.drawPixmap(target, self._icons[0].pixmap(QSize(35,35)), source)
        else:
            qp.drawPixmap(target, self._icons[1].pixmap(QSize(40,40)), source)
        for x in range(len(self._cpu_usage_hist)):
            pct = int(self._cpu_usage_hist[x] / 10.0)
            for y in range (pct):
                if y < 7:
                    color = QtGui.QColor(0, 255, 0)
                else:
                    color = QtGui.QColor(255, 0, 0)
                qp.setBrush(color)
                qp.drawRect(28 - x*2, 30 - (y+1)*2, 2, 2)	

    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)
        self.draw(qp)
        qp.end()
