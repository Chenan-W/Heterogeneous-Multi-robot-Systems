#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/01/04 下午10:59
# @Author : Chenan_Wang
# @File : window_qt_multi
# @Software: PyCharm

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty
import message_filters, rospy, threading
import math, os, sys, time
import numpy as np
import matplotlib.pyplot as plt

# class MainWindow(QWidget):
#     def __init__(self):
#         super(MainWindow, self).__init__()
#         self.setWindowTitle('Bebop Ctrl')
#         self.resize(400, 400)
#
#         layout = QFormLayout()
#         self.setLayout(layout)
#
#         self.linear_x = QLineEdit()
#         self.linear_y = QLineEdit()
#         self.linear_z = QLineEdit()
#
#         btn = QPushButton('TakeOff !')
#         layout.addRow('Vel_X', self.linear_x)
#         layout.addRow('Vel_Y', self.linear_y)
#         layout.addRow('Vel_Z', self.linear_z)
#         layout.addRow(btn)
#
#         btn.clicked.connect(self.click_send)
#
#         topic_name = '/bebop1/cmd_vel'
#         self.publisher = rospy.Publisher(topic_name, Twist, queue_size=100)
#
#     def click_send(self):
#         print 'send'
#
#         linear_x = float(self.linear_x.text())
#         linear_y = float(self.linear_y.text())
#         linear_z = float(self.linear_z.text())
#
#         twist = Twist()
#         twist.linear.x = linear_x
#         twist.linear.y = linear_y
#         twist.linear.z = linear_z
#         self.publisher.publish(twist)


class DataWindow(QWidget):
    def __init__(self):
        super(DataWindow, self).__init__()
        self.uav2_x_last = 0
        self.uav2_y_last = 0
        self.uav2_z_last = 0
        self.uav2_x = 0
        self.uav2_y = 0
        self.uav2_z = 0
        self.uav2_vx = 0
        self.uav2_vy = 0
        self.uav2_vz = 0
        self.uav2_x_err = 0
        self.uav2_y_err = 0
        self.uav2_z_err = 0
        self.uav2_vx_err = 0
        self.uav2_vy_err = 0
        self.control_vx_2 = 0
        self.control_vy_2 = 0
        self.msec_time_2 = 0
        self.vvx_tar_2 = 0
        self.vvy_tar_2 = 0

        self.uav4_x_last = 0
        self.uav4_y_last = 0
        self.uav4_z_last = 0
        self.uav4_x = 0
        self.uav4_y = 0
        self.uav4_z = 0
        self.uav4_vx = 0
        self.uav4_vy = 0
        self.uav4_vz = 0
        self.uav4_x_err = 0
        self.uav4_y_err = 0
        self.uav4_z_err = 0
        self.uav4_vx_err = 0
        self.uav4_vy_err = 0
        self.control_vx_4 = 0
        self.control_vy_4 = 0
        self.msec_time_4 = 0
        self.vvx_tar_4 = 0
        self.vvy_tar_4 = 0

        self.tb1_x = 0
        self.tb1_y = 0
        self.tb2_x = 0
        self.tb2_y = 0
        self.tb3_x = 0
        self.tb3_y = 0
        self.tb4_x = 0
        self.tb4_y = 0

        self.flag_send = 0
        self.flag_draw = 1
        self.flag1 = 0
        self.flag2 = 0
        self.flag3 = 0
        self.flag4 = 0

        update_timer = QTimer(self)
        update_timer.setInterval(16)  # 16ms update -- 60fps
        update_timer.start()

        update_timer.timeout.connect(self.on_update)

        self.setWindowTitle('Bebop Ctrl')
        self.resize(400, 400)

        wlayout = QVBoxLayout()
        layout1 = QFormLayout()
        layout2 = QHBoxLayout()
        layout3 = QHBoxLayout()
        layout4 = QHBoxLayout()
        layout5 = QHBoxLayout()
        layout6 = QHBoxLayout()
        layout7 = QHBoxLayout()
        layout8 = QHBoxLayout()
        layout9 = QHBoxLayout()
        splitter1 = QSplitter(Qt.Horizontal)

        self.linear_x = QLineEdit()
        self.linear_y = QLineEdit()
        self.linear_z = QLineEdit()

        self.lb_x = QLabel()
        self.lb_y = QLabel()
        self.lb_z = QLabel()
        self.lb_linear_x = QLabel()
        self.lb_linear_y = QLabel()
        self.lb_linear_z = QLabel()

        btn_send = QPushButton('Send !')
        btn_stop = QPushButton('Stop !')
        btn_takeoff = QPushButton('TakeOff !')
        btn_land = QPushButton('Land !')
        btn_draw = QPushButton('Draw')
        btn_save = QPushButton('Save')
        label1 = QLabel()
        label1.setText('x')
        label2 = QLabel()
        label2.setText(' y ')
        label3 = QLabel()
        label3.setText(' z ')
        label4 = QLabel()
        label4.setText('  ')
        label_text = QLabel()
        label_text.setText('选择无人机序号：')
        label_text2 = QLabel()
        label_text2.setText('手动输入目标点：')
        label_text3 = QLabel()
        label_text3.setText('手动输入速度指令：')
        ck1 = QCheckBox('UAV_01')
        ck2 = QCheckBox('UAV_02')
        ck3 = QCheckBox('UAV_03')
        ck4 = QCheckBox('UAV_04')

        topleft = QFrame(self)
        topleft.setFrameShape(QFrame.StyledPanel)

        splitter1.addWidget(topleft)
        layout5.addWidget(label_text)
        layout4.addWidget(ck1)
        layout4.addWidget(ck2)
        layout4.addWidget(ck3)
        layout4.addWidget(ck4)

        # self.setLayout(layout1)
        layout6.addWidget(label_text2)
        layout3.addWidget(label1)
        layout3.addWidget(self.linear_x)
        layout3.addWidget(label2)
        layout3.addWidget(self.linear_y)
        layout3.addWidget(label3)
        layout3.addWidget(self.linear_z)
        layout3.addWidget(label4)
        layout3.addWidget(btn_send)
        layout3.addWidget(btn_stop)
        layout7.addWidget(label4)
        layout8.addWidget(label4)
        layout9.addWidget(label4)

        layout1.addRow('当前位置 X：', self.lb_x)
        layout1.addRow('当前位置 Y：', self.lb_y)
        layout1.addRow('当前位置 Z：', self.lb_z)
        layout1.addRow('当前速度 Vx：', self.lb_linear_x)
        layout1.addRow('当前速度 Vy：', self.lb_linear_y)
        layout1.addRow('当前速度 Vz：', self.lb_linear_z)

        layout2.addWidget(btn_takeoff)
        layout2.addWidget(btn_land)
        layout2.addWidget(btn_draw)
        layout2.addWidget(btn_save)

        # wlayout.addLayout(splitter1)
        wlayout.addLayout(layout5)
        wlayout.addLayout(layout4)
        wlayout.addLayout(layout7)
        wlayout.addLayout(layout6)
        wlayout.addLayout(layout3)
        wlayout.addLayout(layout8)
        wlayout.addLayout(layout1)
        wlayout.addLayout(layout9)
        wlayout.addLayout(layout2)
        self.setLayout(wlayout)

        ck1.stateChanged.connect(self.ck1_func)
        ck2.stateChanged.connect(self.ck2_func)
        ck3.stateChanged.connect(self.ck3_func)
        ck4.stateChanged.connect(self.ck4_func)

        btn_send.clicked.connect(self.a)
        btn_stop.clicked.connect(self.data_stop)
        btn_takeoff.clicked.connect(self.takeoff_send)
        btn_land.clicked.connect(self.land_send)
        btn_draw.clicked.connect(self.a_draw)
        btn_save.clicked.connect(self.draw_save)

        # pose_topic_name = '/vrpn_client_node/uav1219_2/pose'
        # rospy.Subscriber(pose_topic_name, PoseStamped, self.pose_callback, queue_size=1)
        
        UAV_01 = message_filters.Subscriber('/vrpn_client_node/UAV_01/pose', PoseStamped, queue_size=1)
        UAV_02 = message_filters.Subscriber('/vrpn_client_node/uav1213_1/pose', PoseStamped, queue_size=1)
        UAV_03 = message_filters.Subscriber('/vrpn_client_node/UAV_03/pose', PoseStamped, queue_size=1)
        UAV_04 = message_filters.Subscriber('/vrpn_client_node/uav1219_2/pose', PoseStamped, queue_size=1)
        AGV_01 = message_filters.Subscriber('/vrpn_client_node/tb3/pose', PoseStamped, queue_size=1)
        # AGV_02 = message_filters.Subscriber('/vrpn_client_node/tb2/pose', PoseStamped, queue_size=1)
        # AGV_03 = message_filters.Subscriber('/vrpn_client_node/tb3/pose', PoseStamped, queue_size=1)
        # AGV_04 = message_filters.Subscriber('/vrpn_client_node/tb4/pose', PoseStamped, queue_size=1)
        sync = message_filters.ApproximateTimeSynchronizer([UAV_02, UAV_04, AGV_01], 10, 0.1, allow_headerless=True)
        sync.registerCallback(self.multi_callback)

    def GetNowTime(self):
        return time.strftime("%m%d%H%M%S", time.localtime(time.time()))

    def multi_callback(self, UAV_02, UAV_04, AGV_01):
        if self.flag2 == 1:
            if not isinstance(UAV_02, PoseStamped):
                return
            self.tb1_x = - AGV_01.pose.position.x
            self.tb1_y = AGV_01.pose.position.y
            # self.tb2_x = - AGV_02.pose.position.x
            # self.tb2_y = AGV_02.pose.position.y
            # self.tb3_x = - AGV_03.pose.position.x
            # self.tb3_y = AGV_03.pose.position.y
            # self.tb4_x = - AGV_04.pose.position.x
            # self.tb4_y = AGV_04.pose.position.y

            uav2_x = - UAV_02.pose.position.x
            uav2_y = UAV_02.pose.position.y
            uav2_z = UAV_02.pose.position.z - 58
            last_time_2 = self.msec_time_2
            nsec_time_2 = UAV_02.header.stamp.nsecs
            self.msec_time_2 = nsec_time_2 / 1000000
            if (self.msec_time_2 - last_time_2) > 0:
                time_gap_2 = round((self.msec_time_2 - last_time_2), 4)
            else:
                time_gap_2 = round((self.msec_time_2 - last_time_2 + 1000), 4)
            self.uav2_x = round(uav2_x, 1)
            self.uav2_y = round(uav2_y, 1)
            self.uav2_z = round(uav2_z, 1)
            uav2_vx = (self.uav2_x - self.uav2_x_last) / time_gap_2
            uav2_vy = (self.uav2_y - self.uav2_y_last) / time_gap_2
            uav2_vz = (self.uav2_z - self.uav2_z_last) / time_gap_2
            self.uav2_vx = round(uav2_vx, 8)
            self.uav2_vy = round(uav2_vy, 8)
            self.uav2_vz = round(uav2_vz, 8)
            self.lb_x.setText(str(self.uav2_x))
            self.lb_y.setText(str(self.uav2_y))
            self.lb_z.setText(str(self.uav2_z))
            self.lb_linear_x.setText(str(self.uav2_vx))
            self.lb_linear_y.setText(str(self.uav2_vy))
            self.lb_linear_z.setText(str(self.uav2_vz))
            self.uav2_x_last = self.uav2_x
            self.uav2_y_last = self.uav2_y
            self.uav2_z_last = self.uav2_z

        if self.flag4 == 1:
            if not isinstance(UAV_04, PoseStamped):
                return
            self.tb1_x = - AGV_01.pose.position.x
            self.tb1_y = AGV_01.pose.position.y

            uav4_x = - UAV_04.pose.position.x
            uav4_y = UAV_04.pose.position.y
            uav4_z = UAV_04.pose.position.z - 58

            last_time_4 = self.msec_time_4
            nsec_time_4 = UAV_04.header.stamp.nsecs
            self.msec_time_4 = nsec_time_4 / 1000000
            if (self.msec_time_4 - last_time_4) > 0:
                time_gap_4 = round((self.msec_time_4 - last_time_4), 4)
            else:
                time_gap_4 = round((self.msec_time_4 - last_time_4 + 1000), 4)
            self.uav4_x = round(uav4_x, 1)
            self.uav4_y = round(uav4_y, 1)
            self.uav4_z = round(uav4_z, 1)
            uav4_vx = (self.uav4_x - self.uav4_x_last) / time_gap_4
            uav4_vy = (self.uav4_y - self.uav4_y_last) / time_gap_4
            uav4_vz = (self.uav4_z - self.uav4_z_last) / time_gap_4
            self.uav4_vx = round(uav4_vx, 8)
            self.uav4_vy = round(uav4_vy, 8)
            self.uav4_vz = round(uav4_vz, 8)
            self.lb_x.setText(str(self.uav4_x))
            self.lb_y.setText(str(self.uav4_y))
            self.lb_z.setText(str(self.uav4_z))
            self.lb_linear_x.setText(str(self.uav4_vx))
            self.lb_linear_y.setText(str(self.uav4_vy))
            self.lb_linear_z.setText(str(self.uav4_vz))
            self.uav4_x_last = self.uav4_x
            self.uav4_y_last = self.uav4_y
            self.uav4_z_last = self.uav4_z

    def ck1_func(self, state):
        if state == 2:
            print 'Select UAV_01 !!!'
            self.flag1 = 1
        elif state == 0:
            print "Deselect UAV_01 !!!"
            self.flag1 = 0
        print 'Flag1 =', self.flag1

    def ck2_func(self, state):
        if state == 2:
            print 'Select UAV_02 !!!'
            self.flag2 = 1
        elif state == 0:
            print "Deselect UAV_02 !!!"
            self.flag2 = 0
        print 'Flag2 =', self.flag2

    def ck3_func(self, state):
        if state == 2:
            print 'Select UAV_03 !!!'
            self.flag3 = 1
        elif state == 0:
            print "Deselect UAV_03 !!!"
            self.flag3 = 0
        print 'Flag3 =', self.flag3

    def ck4_func(self, state):
        if state == 2:
            print 'Select UAV_04 !!!'
            self.flag4 = 1
        elif state == 0:
            print "Deselect UAV_04 !!!"
            self.flag4 = 0
        print 'Flag4 =', self.flag4

    def takeoff_send(self):
        print 'UAV Takeoff !!!'
        if self.flag1 == 1:
            takeoff_pub = rospy.Publisher('/bebop1/takeoff', Empty, queue_size=10)
            takeoff_pub.publish(Empty())
        if self.flag2 == 1:
            takeoff_pub = rospy.Publisher('/bebop2/takeoff', Empty, queue_size=10)
            takeoff_pub.publish(Empty())    
        if self.flag3 == 1:
            takeoff_pub = rospy.Publisher('/bebop3/takeoff', Empty, queue_size=10)
            takeoff_pub.publish(Empty())
        if self.flag4 == 1:
            takeoff_pub = rospy.Publisher('/bebop4/takeoff', Empty, queue_size=10)
            takeoff_pub.publish(Empty())

    def land_send(self):
        print 'UAV Land !!!'
        if self.flag1 == 1:
            land_pub = rospy.Publisher('/bebop1/land', Empty, queue_size=10)
            land_pub.publish(Empty())
        if self.flag2 == 1:
            land_pub = rospy.Publisher('/bebop2/land', Empty, queue_size=10)
            land_pub.publish(Empty())
        if self.flag3 == 1:
            land_pub = rospy.Publisher('/bebop3/land', Empty, queue_size=10)
            land_pub.publish(Empty())
        if self.flag4 == 1:
            land_pub = rospy.Publisher('/bebop4/land', Empty, queue_size=10)
            land_pub.publish(Empty())

    def a_draw(self):
        t2 = threading.Thread(target=self.data_draw, name='t2')
        t2.start()

    def data_draw(self):
        self.flag_draw = 1
        fig, ax1 = plt.subplots(figsize=(4, 7))
        # fig2 = plt.figure()
        # ax2 = fig2.add_subplot(311)
        # ax3 = fig2.add_subplot(312)
        # ax8 = fig2.add_subplot(313)
        fig3 = plt.figure()
        ax5 = fig3.add_subplot(311)
        ax6 = fig3.add_subplot(312)
        ax4 = fig3.add_subplot(313)
        now_x2 = []
        now_y2 = []
        now_x4 = []
        now_y4 = []
        controlv2 = []
        controlv4 = []
        x_tar2 = []
        y_tar2 = []
        x_tar4 = []
        y_tar4 = []
        vvvx2 = []
        vvvy2 = []
        vvvx4 = []
        vvvy4 = []
        vx_tar2 = []
        vy_tar2 = []
        vx_tar4 = []
        vy_tar4 = []

        t = []
        i = 0
        while True:
            if self.flag_draw == 1:
                if self.flag2 == 1 and self.flag4 == 1:
                    t.append(i)

                    x_target_2 = float(self.linear_x.text())
                    y_target_2 = float(self.linear_y.text())
                    # x_target_2 = self.tb1_x
                    # y_target_2 = self.tb1_y
                    x2 = self.uav2_x
                    y2 = self.uav2_y
                    now_x2.append(x2)
                    now_y2.append(y2)
                    x_tar2.append(x_target_2)
                    y_tar2.append(y_target_2)
                    vx2 = self.uav2_vx
                    vy2 = self.uav2_vy
                    vvvx2.append(vx2)
                    vvvy2.append(vy2)
                    vx_tar2.append(self.vvx_tar_2)
                    vy_tar2.append(self.vvy_tar_2)
                    control_vx_2 = self.control_vx_2
                    controlv2.append(control_vx_2)

                    x_target_4 = self.uav2_x - 500
                    y_target_4 = self.uav2_x
                    x4 = self.uav4_x
                    y4 = self.uav4_y
                    now_x4.append(x4)
                    now_y4.append(y4)
                    x_tar4.append(x_target_4)
                    y_tar4.append(y_target_4)
                    vx4 = self.uav4_vx
                    vy4 = self.uav4_vy
                    vvvx4.append(vx4)
                    vvvy4.append(vy4)
                    vx_tar4.append(self.vvx_tar_4)
                    vy_tar4.append(self.vvy_tar_4)
                    control_vx_4 = self.control_vx_4
                    controlv4.append(control_vx_4)

                    ax1.cla()
                    # ax2.cla()
                    # ax3.cla()
                    # ax4.cla()
                    ax5.cla()
                    ax6.cla()

                    ax1.set_title("UAVs")
                    ax1.set_xlabel("y / mm")
                    ax1.set_ylabel("x / mm")
                    ax1.set_xlim(-1000, 1000)
                    ax1.set_ylim(-1800, 1800)
                    ax1.grid()
                    ax1.scatter(y2, x2, s=10, c='g')
                    ax1.scatter(y4, x4, s=10, c='g')
                    ax1.scatter(y_target_2, x_target_2, s=10, c='r')

                    ax5.set_ylim(-1000, 1000)
                    ax5.plot(t, x_tar2)
                    ax5.plot(t, now_x2)
                    ax6.set_xlabel("t")
                    ax6.set_ylabel("y / mm")
                    ax6.set_ylim(-1000, 1000)
                    ax6.plot(t, y_tar2)
                    ax6.plot(t, now_y2)
                    ax4.plot(t, controlv2, label='v')
                    i += 1
                    plt.pause(0.01)

                if self.flag2 == 1 and self.flag4 != 1:
                    t.append(i)

                    x_target_2 = float(self.linear_x.text())
                    y_target_2 = float(self.linear_y.text())

                    # x_target_2 = self.tb1_x
                    # y_target_2 = self.tb1_y

                    x2 = self.uav2_x
                    y2 = self.uav2_y
                    now_x2.append(x2)
                    now_y2.append(y2)
                    x_tar2.append(x_target_2)
                    y_tar2.append(y_target_2)
                    vx2 = self.uav2_vx
                    vy2 = self.uav2_vy
                    vvvx2.append(vx2)
                    vvvy2.append(vy2)
                    vx_tar2.append(self.vvx_tar_2)
                    vy_tar2.append(self.vvy_tar_2)
                    control_vx_2 = self.control_vx_2
                    controlv2.append(control_vx_2)

                    ax1.cla()
                    # ax2.cla()
                    # ax3.cla()
                    # ax4.cla()
                    ax5.cla()
                    ax6.cla()

                    ax1.set_title("UAVs")
                    ax1.set_xlabel("y / mm")
                    ax1.set_ylabel("x / mm")
                    ax1.set_xlim(-1000, 1000)
                    ax1.set_ylim(-1800, 1800)
                    ax1.grid()
                    ax1.scatter(y2, x2, marker='x', s=50, c='g')
                    ax1.scatter(y_target_2, x_target_2, marker='*', s=80, c='r')
                    
                    ax5.set_ylim(-1000, 1000)
                    ax5.plot(t, x_tar2)
                    ax5.plot(t, now_x2)
                    ax6.set_xlabel("t")
                    ax6.set_ylabel("y / mm")
                    ax6.set_ylim(-1000, 1000)
                    ax6.plot(t, y_tar2)
                    ax6.plot(t, now_y2)
                    ax4.plot(t, controlv2, label='v')
                    i += 1
                    plt.pause(0.01)

                if self.flag4 == 1 and self.flag2 != 1:
                    t.append(i)
                    x_target_4 = float(self.linear_x.text())
                    y_target_4 = float(self.linear_y.text())
                    x4 = self.uav4_x
                    y4 = self.uav4_y
                    now_x4.append(x4)
                    now_y4.append(y4)
                    x_tar4.append(x_target_4)
                    y_tar4.append(y_target_4)
                    vx4 = self.uav4_vx
                    vy4 = self.uav4_vy
                    vvvx4.append(vx4)
                    vvvy4.append(vy4)
                    vx_tar4.append(self.vvx_tar_4)
                    vy_tar4.append(self.vvy_tar_4)
                    control_vx_4 = self.control_vx_4
                    controlv4.append(control_vx_4)
                    ax1.cla()
                    # ax2.cla()
                    # ax3.cla()
                    # ax4.cla()
                    ax5.cla()
                    ax6.cla()
                    ax1.set_title("UAVs")
                    ax1.set_xlabel("y / mm")
                    ax1.set_ylabel("x / mm")
                    ax1.set_xlim(-1000, 1000)
                    ax1.set_ylim(-1800, 1800)
                    ax1.grid()
                    ax1.scatter(y4, x4, s=10, c='g')
                    ax1.scatter(y_target_4, x_target_4, s=10, c='r')

                    # ax2.set_xlabel("t")
                    # ax2.set_ylabel("vx (mm/ms)")
                    # ax2.set_ylim(-1, 1)
                    # ax2.plot(t, vx_tar)
                    # ax2.plot(t, vvvx)
                    # ax3.set_xlabel("t")
                    # ax3.set_ylim(-1, 1)
                    # ax3.set_ylabel("vy (mm/ms)")
                    # ax3.plot(t, vy_tar)
                    # ax3.plot(t, vvvy)

                    ax4.plot(t, controlv4, label='v')
                    ax5.set_xlabel("t")
                    ax5.set_ylabel("x / mm")
                    ax5.set_ylim(-1000, 1000)
                    ax5.plot(t, x_tar4)
                    ax5.plot(t, now_x4)
                    ax6.set_xlabel("t")
                    ax6.set_ylabel("y / mm")
                    ax6.set_ylim(-1000, 1000)
                    ax6.plot(t, y_tar4)
                    ax6.plot(t, now_y4)
                    ax4.plot(t, controlv4)
                    i += 1
                    plt.pause(0.01)

            else:
                time = self.GetNowTime()
                if self.flag2 == 1:
                    x2_target = np.array([x_tar2]).T
                    y2_target = np.array([y_tar2]).T
                    now_xx2 = np.array([now_x2]).T
                    now_yy2 = np.array([now_y2]).T
                    vx2_target = np.array([vx_tar2]).T
                    vy2_target = np.array([vy_tar2]).T
                    now_vvvx = np.array([vvvx2]).T
                    now_vvvy = np.array([vvvy2]).T
                    t_list = np.array([t]).T
                    control_uav1 = np.array([controlv2]).T
                    # print x2_target
                    uav2_data_save = np.concatenate((t_list, x2_target, y2_target, now_xx2, now_yy2, vx2_target, vy2_target, now_vvvx, now_vvvy, control_uav1), axis=1)
                    np.savetxt("/home/hlzy/chenan_wang/mulit-bebop_ws/uav2_data_"+time+".txt", uav2_data_save, fmt='%f', delimiter=', ')
                    plt.savefig("/home/hlzy/chenan_wang/mulit-bebop_ws/uav2_"+time+".png")
                if self.flag4 == 1:
                    x4_target = np.array([x_tar4]).T
                    y4_target = np.array([y_tar4]).T
                    now_xx4 = np.array([now_x4]).T
                    now_yy4 = np.array([now_y4]).T
                    vx4_target = np.array([vx_tar4]).T
                    vy4_target = np.array([vy_tar4]).T
                    now_vvvx = np.array([vvvx4]).T
                    now_vvvy = np.array([vvvy4]).T
                    t_list = np.array([t]).T
                    control_uav1 = np.array([controlv4]).T
                    # print x4_target
                    uav4_data_save = np.concatenate((t_list, x4_target, y4_target, now_xx4, now_yy4, vx4_target, vy4_target, now_vvvx, now_vvvy, control_uav1), axis=1)
                    np.savetxt("/home/hlzy/chenan_wang/mulit-bebop_ws/uav4_data_"+time+".txt", uav4_data_save, fmt='%f', delimiter=', ')
                    plt.savefig("/home/hlzy/chenan_wang/mulit-bebop_ws/uav4_"+time+".png")
                plt.close(fig)
                # plt.close(fig2)
                plt.close(fig3)
                break

    def a(self):
        t = threading.Thread(target=self.data_send, name='t')
        t.start()

    def data_stop(self):
        self.flag_send = 0

    def draw_save(self):
        self.flag_draw = 0

    def data_send(self):
        print 'Send Data !!!'
        self.flag_send = 1
        while True:
            if self.flag_send == 1:
                rospy.sleep(0.2)
                if self.flag1 == 1:
                    topic_name = '/bebop1/cmd_vel'
                    data_pub = rospy.Publisher(topic_name, Twist, queue_size=100)
                    linear_x = float(self.linear_x.text())
                    linear_y = float(self.linear_y.text())
                    linear_z = float(self.linear_z.text())
                    twist = Twist()
                    twist.linear.x = linear_x
                    twist.linear.y = linear_y
                    twist.linear.z = linear_z
                    data_pub.publish(twist)

                if self.flag2 == 1:
                    topic_name = '/bebop2/cmd_vel'
                    data_pub = rospy.Publisher(topic_name, Twist, queue_size=100)
                    x_target_2 = float(self.linear_x.text())
                    y_target_2 = float(self.linear_y.text())
                    z_target_2 = float(self.linear_z.text())

                    # x_target_2 = self.tb1_x - 300
                    # y_target_2 = self.tb1_y + 500

                    # x_target_2 = self.tb1_x # + 400
                    # y_target_2 = self.tb1_y + 600

                    x_last_err = self.uav2_x_err
                    y_last_err = self.uav2_y_err
                    z_last_err = self.uav2_z_err
                    self.uav2_x_err = x_target_2 - self.uav2_x
                    self.uav2_y_err = y_target_2 - self.uav2_y
                    self.uav2_z_err = z_target_2 - self.uav2_z
                    sum_err = math.sqrt(math.pow(self.uav2_x_err, 2) + math.pow(self.uav2_y_err, 2))

                    linear_x = 0.0014 * self.uav2_x_err + 0.0012 * (self.uav2_x_err - x_last_err)
                    linear_y = 0.0014 * self.uav2_y_err + 0.0012 * (self.uav2_y_err - y_last_err)
                    # linear_x = 0.0011 * self.uav2_x_err + 0.0011 * (self.uav2_x_err - x_last_err)
                    # linear_y = 0.0011 * self.uav2_y_err + 0.0011 * (self.uav2_y_err - y_last_err)
                    # print 0.005*(self.uav2_x_err - x_last_err), 0.005*(self.uav2_y_err - y_last_err)

                    # if linear_x > 0.3:
                    #     linear_x = 0.3
                    # if linear_x < -0.3:
                    #     linear_x = -0.3
                    # if linear_y > 0.3:
                    #     linear_y = 0.3
                    # if linear_y < -0.3:
                    #     linear_y = -0.3

                    self.vvx_tar_2 = linear_x
                    self.vvy_tar_2 = linear_y
                    # linear_x = float(self.linear_x.text())
                    # linear_y = float(self.linear_y.text())

                    vx_last_err = self.uav2_vx_err
                    vy_last_err = self.uav2_vy_err

                    self.uav2_vx_err = linear_x - self.uav2_vx
                    self.uav2_vy_err = linear_y - self.uav2_vy

                    if sum_err <= 20:
                        angle_x = 0
                        angle_y = 0
                    else:
                        angle_x = -0.2 * self.uav2_vx_err + 0.012 * (self.uav2_vx_err - vx_last_err)
                        angle_y = -0.2 * self.uav2_vy_err + 0.012 * (self.uav2_vx_err - vy_last_err)

                    twist = Twist()
                    twist.linear.x = angle_x
                    twist.linear.y = angle_y
                    twist.linear.z = 0

                    if twist.linear.x > 0.3:
                        twist.linear.x = 0.3
                    if twist.linear.x < -0.3:
                        twist.linear.x = -0.3
                    if twist.linear.y > 0.3:
                        twist.linear.y = 0.3
                    if twist.linear.y < -0.3:
                        twist.linear.y = -0.3

                    if abs(self.uav2_x) >= 9999:
                        twist.linear.x = 0
                        twist.linear.y = 0

                    self.control_vx_2 = twist.linear.x
                    self.control_vy_2 = twist.linear.y
                    # print 'Control pub: ', twist.linear.x, twist.linear.y
                    data_pub.publish(twist)

                if self.flag3 == 1:
                    topic_name = '/bebop3/cmd_vel'
                    data_pub = rospy.Publisher(topic_name, Twist, queue_size=100)
                    linear_x = float(self.linear_x.text())
                    linear_y = float(self.linear_y.text())
                    linear_z = float(self.linear_z.text())
                    twist = Twist()
                    twist.linear.x = linear_x
                    twist.linear.y = linear_y
                    twist.linear.z = linear_z
                    data_pub.publish(twist)

                if self.flag4 == 1:
                    topic_name = '/bebop4/cmd_vel'
                    data_pub = rospy.Publisher(topic_name, Twist, queue_size=100)
                    # x_target_4 = float(self.linear_x.text())
                    # y_target_4 = float(self.linear_y.text())
                    # x_target_4 = float(self.linear_x.text()) - 600
                    # y_target_4 = float(self.linear_y.text())
                    z_target_4 = float(self.linear_z.text())

                    x_target_4 = self.tb1_x  #  400
                    y_target_4 = self.tb1_y - 600
                    print(x_target_4, y_target_4)

                    x_last_err = self.uav4_x_err
                    y_last_err = self.uav4_y_err
                    self.uav4_x_err = x_target_4 - self.uav4_x
                    self.uav4_y_err = y_target_4 - self.uav4_y
                    self.uav4_z_err = z_target_4 - self.uav4_z
                    sum_err = math.sqrt(math.pow(self.uav4_x_err, 4) + math.pow(self.uav4_y_err, 4))

                    linear_x = 0.0013 * self.uav4_x_err + 0.0011 * (self.uav4_x_err - x_last_err)
                    linear_y = 0.0013 * self.uav4_y_err + 0.0011 * (self.uav4_y_err - y_last_err)

                    if linear_x > 0.3:
                        linear_x = 0.3
                    if linear_x < -0.3:
                        linear_x = -0.3
                    if linear_y > 0.3:
                        linear_y = 0.3
                    if linear_y < -0.3:
                        linear_y = -0.3

                    self.vvx_tar_4 = linear_x
                    self.vvy_tar_4 = linear_y

                    vx_last_err = self.uav4_vx_err
                    vy_last_err = self.uav4_vy_err

                    self.uav4_vx_err = linear_x - self.uav4_vx
                    self.uav4_vy_err = linear_y - self.uav4_vy

                    if sum_err <= 30:
                        angle_x = 0
                        angle_y = 0
                    else:
                        angle_x = -0.22 * self.uav4_vx_err + 0.01 * (self.uav4_vx_err - vx_last_err)
                        angle_y = -0.22 * self.uav4_vy_err + 0.01 * (self.uav4_vx_err - vy_last_err)

                    twist = Twist()
                    twist.linear.x = angle_x
                    twist.linear.y = angle_y
                    twist.linear.z = 0

                    if twist.linear.x > 0.3:
                        twist.linear.x = 0.3
                    if twist.linear.x < -0.3:
                        twist.linear.x = -0.3
                    if twist.linear.y > 0.3:
                        twist.linear.y = 0.3
                    if twist.linear.y < -0.3:
                        twist.linear.y = -0.3

                    # if twist.linear.x > 0.08:
                    #     twist.linear.x = 0.08
                    # if twist.linear.x < -0.08:
                    #     twist.linear.x = -0.08
                    # if twist.linear.y > 0.08:
                    #     twist.linear.y = 0.08
                    # if twist.linear.y < -0.08:
                    #     twist.linear.y = -0.08

                    if self.uav4_x >= 9999:
                        twist.linear.x = 0
                        twist.linear.y = 0
                    if self.uav4_x <= -9999:
                        twist.linear.x = 0
                        twist.linear.y = 0

                    self.control_vx_4 = twist.linear.x
                    self.control_vy_4 = twist.linear.y
                    data_pub.publish(twist)
            else:
                break
        print("Stop !!!")

    def on_update(self):
        # render UI
        self.update()
        if rospy.is_shutdown():
            self.close()
