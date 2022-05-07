#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/01/04 上午07:57
# @Author : Chenan_Wang
# @File : bebop_ctrl.py
# @Software: PyCharm

from PyQt5.QtWidgets import *
from window_qt_multi import DataWindow
import rospy
import sys


if __name__ == '__main__':

    rospy.init_node('turtle_ctrl_node')

    app = QApplication(sys.argv)

    window = DataWindow()
    window.show()

    sys.exit(app.exec_())

