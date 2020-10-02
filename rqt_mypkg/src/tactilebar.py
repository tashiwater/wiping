#!/usr/bin/env python
# -*- coding: utf-8 -*-


# from qt_gui.plugin import Plugin
# from python_qt_binding import loadUi
# import sys


# class TactileBar(Plugin):
#     def __init__(self, context):
#         super(self.__class__, self).__init__(context)
#         self.setObjectName('TactileBar')
#         self._touch = None

#         # Create QWidget
#         self._widget = QWidget()

#         # load ui
#         ui_file = os.path.join(rospkg.RosPack().get_path('torobo_gui'), 'resource', 'torobo_whole_body_manager.ui')
#         loadUi(ui_file, self._widget)
#         self._widget.setObjectName('ToroboWholeBodyManagerUi')

import sys
from PyQt5.QtWidgets import QWidget, QCheckBox, QApplication
from PyQt5.QtCore import Qt, QTimer
from PyQt5 import uic

import rospy
from std_msgs.msg import Float32MultiArray
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)


class TactileBar(QWidget):

    def __init__(self):
        super(TactileBar, self).__init__()
        ui_path = "/home/assimilation/TAKUMI_SHIMIZU/wiping_ws/src/wiping/rqt_mypkg/ui/tactilebar.ui"
        uic.loadUi(ui_path, self)
        rospy.Subscriber("/touchence/sensor_data",
                         Float32MultiArray, self.TouchSensorCallback)

        self._default_tactilebar = """
        QProgressBar{
            border: 2px solid grey;
            border-radius: 5px;
            text-align: center
        }

        QProgressBar::chunk {
            background-color: lightblue;
            width: 10px;
            margin: 1px;
        }"""
        self._high_tactilebar = """
        QProgressBar{
            border: 2px solid grey;
            border-radius: 5px;
            text-align: center
        }

        QProgressBar::chunk {
            background-color: red;
            width: 10px;
            margin: 1px;
        }
        """
        self._bar_update_count = 0
        self._touchbar_list = [self.progressBar,
                               self.progressBar_2, self.progressBar_3]
        self._bar_state = [None for i in range(len(self._touchbar_list))]
        self._touch = None

        self.timer = QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_bar)
        self.timer.start()

    def update_bar(self):
        if rospy.is_shutdown():
            self.exec_()
        if self._touch is None:
            return
        # enumerate(self._touchbar_list):
        for i in range(len(self._touchbar_list)):
            targetbar = self._touchbar_list[i]
            max_data = max(self._touch[i*4: i*4 + 4])*100
            targetbar.setValue(max_data)
            old_state = self._bar_state[i]
            if max_data > 15:
                state = "high"
                if old_state != state:
                    targetbar.setStyleSheet(self._high_tactilebar)
            else:
                state = "default"
                if old_state != state:
                    targetbar.setStyleSheet(self._default_tactilebar)
            self._bar_state[i] = state

    def TouchSensorCallback(self, data):
        self._touch = data.data


if __name__ == '__main__':

    rospy.init_node('tactile_bar', anonymous=True)
    app = QApplication(sys.argv)
    ex = TactileBar()
    ex.show()
    app.exec_()
    # sys.exit()
