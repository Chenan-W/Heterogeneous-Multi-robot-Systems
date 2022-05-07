#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2020/11/30 上午12:05
# @Author :
# @File : robot_follow04.py
# @Software: CLion


import rospy
import math
import time
import nav_msgs.msg
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

# 定义全局变量
x1 = 0
y1 = 0
ww1 = 0
xx1 = 0
yy1 = 0
zz1 = 0

x2 = 0
y2 = 0
ww2 = 0
xx2 = 0
yy2 = 0
zz2 = 0

x4 = 0
y4 = 0
ww4 = 0
xx4 = 0
yy4 = 0
zz4 = 0

yaw_t = 0
liner_speed = 0
angular_speed = 0
liner_speed_old = 0
angular_speed_old = 0
final_flag = 0
X_t = 0
Y_t = 0
X_t_Pre = 0
Y_t_Pre = 0
triger = 0
r = 0
X_t_old = 0
Y_t_old = 0
Friquent = 30

# 机器人参数
m = 100
l1 = 1
l2 = 1
d = 10
u = 0
beta = 0


def Trans_robot_pose1(msg):
    # 位置坐标声明
    global x1
    global y1
    global ww1
    global xx1
    global yy1
    global zz1
    x1 = -msg.pose.position.x
    y1 = msg.pose.position.y
    ww1 = msg.pose.orientation.w
    xx1 = msg.pose.orientation.x
    yy1 = msg.pose.orientation.y
    zz1 = msg.pose.orientation.z
    return ww1, yy1, zz1, xx1, x1, y1


def Trans_robot_pose2(msg):
    # 位置坐标声明
    global x2
    global y2
    global ww2
    global xx2
    global yy2
    global zz2
    x2 = -msg.pose.position.x
    y2 = msg.pose.position.y
    ww2 = msg.pose.orientation.w
    xx2 = msg.pose.orientation.x
    yy2 = msg.pose.orientation.y
    zz2 = msg.pose.orientation.z
    return ww2, yy2, zz2, xx2, x2, y2


def Trans_robot_pose4(msg):
    # 位置坐标声明
    global x4
    global y4
    global ww4
    global xx4
    global yy4
    global zz4
    x4 = -msg.pose.position.x
    y4 = msg.pose.position.y
    ww4 = msg.pose.orientation.w
    xx4 = msg.pose.orientation.x
    yy4 = msg.pose.orientation.y
    zz4 = msg.pose.orientation.z
    return ww4, yy4, zz4, xx4, x4, y4


if __name__ == '__main__':
    rospy.init_node('item1')
    turtle_vel = rospy.Publisher('/tb1/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(Friquent)  # 循环执行，更新频率是10hz

    while not rospy.is_shutdown():
        msg = Twist()
        (roll, pitch, yaw) = euler_from_quaternion([xx1, yy1, zz1, ww1])
        # chanange xyz
        yaw = -yaw
        yaw = yaw + math.pi
        if (yaw >= 1.5 * math.pi) and (yaw <= 2 * math.pi):
            yaw = yaw - 1.5 * math.pi
        else:
            yaw = yaw + 0.5 * math.pi

        # X_t = -400
        # Y_t = -400  # type: int

        X_t = x2 + 600
        Y_t = y2 + 400

        D_err = math.sqrt(math.pow((X_t - x1), 2) + math.pow((Y_t - y1), 2))
        u = D_err

        if (Y_t - y1) == 0 and (X_t - x1) > 0:
            yaw_t = 0
        if (Y_t - y1) > 0 and (X_t - x1) > 0:
            yaw_t = math.atan((Y_t - y1) / (X_t - x1))
        if (Y_t - y1) > 0 and (X_t - x1) == 0:
            yaw_t = 0.5 * math.pi
        if (Y_t - y1) > 0 and (X_t - x1) < 0:
            yaw_t = math.atan((Y_t - y1) / (X_t - x1)) + math.pi
        if (Y_t - y1) == 0 and (X_t - x1) < 0:
            yaw_t = math.pi
        if (Y_t - y1) < 0 and (X_t - x1) < 0:
            yaw_t = math.atan((Y_t - y1) / (X_t - x1)) + math.pi
        if (Y_t - y1) < 0 and (X_t - x1) == 0:
            yaw_t = 1.5 * math.pi
        if (Y_t - y1) < 0 and (X_t - x1) > 0:
            yaw_t = math.atan((Y_t - y1) / (X_t - x1)) + 2 * math.pi

        Theta_err = yaw_t - yaw
        if Theta_err < -math.pi:
            Theta_err = Theta_err + 2 * math.pi
        if Theta_err > math.pi:
            Theta_err = Theta_err - 2 * math.pi

        liner_speed = abs(u * math.cos(Theta_err) * 0.0004)
        angular_speed = 0.0005 * (12 * u * math.sin(Theta_err) * d * 0.1) / (math.pow((l1), 2) + math.pow((l2), 2))
        # print(liner_speed)
        if liner_speed > 0.18:
            liner_speed = 0.18
        if liner_speed < 0.05:
            liner_speed = 0.05

        if (D_err > 0) and (D_err < 80):
            liner_speed = 0
            triger += 1
        else:
            triger = 0

        if triger > 10:
            liner_speed = 0
            angular_speed = 1.0 * (math.pi - yaw)
            print(angular_speed)

        if angular_speed > 1.5:
            angular_speed = 1.5
        if angular_speed < -1.5:
            angular_speed = -1.5
        if abs(angular_speed) < 0.1:
            angular_speed = 0

        # if ((D_err > 0) and (D_err < 20)):
        #     final_flag = 1
        # if (final_flag == 1):
        #     liner_speed = 0
        #     angular_speed = 0.8 * (0.5 * math.pi - yaw)

        msg.linear.x = liner_speed
        msg.angular.z = angular_speed
        # print(x1, y1, yaw, msg.linear.x, msg.angular.z)
        # print liner_speed
        liner_speed_old = liner_speed
        angular_speed_old = angular_speed
        X_t_old = X_t
        Y_t_old = Y_t
        turtle_vel.publish(msg)
        rospy.Subscriber('/vrpn_client_node/tb1/pose', PoseStamped, Trans_robot_pose1, queue_size=1)
        rospy.Subscriber('/vrpn_client_node/tb3/pose', PoseStamped, Trans_robot_pose2, queue_size=1)
        rospy.Subscriber('/vrpn_client_node/tb4/pose', PoseStamped, Trans_robot_pose4, queue_size=1)

        rate.sleep()  # 以固定频率执行
    rospy.spin()  # 保持节点运行，直到节点关闭

    # include "tf/transform_datatypes.h"//转换函数头文件
# include <nav_msgs/Odometry.h>//里程计信息格式
