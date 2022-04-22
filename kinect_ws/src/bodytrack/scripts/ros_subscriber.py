#!/usr/bin/env python
# -*- coding: utf-8 -*-
# subscribe角度信息用于机械臂交互

import rospy
from bodytrack.msg import angle
import rtde_control
import rtde_receive
from robotiq_gripper_control import RobotiqGripper
import math
import time

# address of ur10e
rtde_c = rtde_control.RTDEControlInterface("192.168.56.2")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.56.2")

# 用于判断是否启动交互
startFlag = False
rad = math.pi / 180.0

gripper = RobotiqGripper(rtde_c)

# Activate the gripper and initialize force and speed
gripper.activate()  # returns to previous position after activation
gripper.set_force(50)  # from 0 to 100 %
gripper.set_speed(100)  # from 0 to 100 %
gripper.open()


def callback(msg):
    # Parameters
    acceleration = 0.1
    dt = 1.0 / 500  # 2ms
    joint_q_initial = [1.66 * rad, -83.56 * rad, -112.58 * rad,
                       -77.05 * rad, 90.66 * rad, 91.75 * rad]

    joint_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # speed of ur10e
    joint_q_speed = 0.5

    global startFlag

    time.sleep(0.1)

    # 如果没有检测到人体，机械臂回到原位
    if msg.stoptrack == 0:
        startFlag = False
        rtde_c.moveJ(joint_q_initial, joint_q_speed, acceleration)
    elif 70.0 < msg.wes < 110.0 and -10 < msg.turn < 10:
        startFlag = True

    # 用于限制机械臂上下移动范围，上面范围大，下面范围小
    if startFlag:
        angle1_turn = (1.66 + msg.angle1 * 1 / 4) * rad
        if msg.angle2 < 110.0:
            angle2_down = msg.angle2 * 2 / 11
            angle2_down_revert = - (angle2_down - 20)
            angle2_turn = (-83.56 - angle2_down_revert) * rad
        else:
            angle2_turn = (-83.56 + ((msg.angle2 - 100) * 1 / 4)) * rad
        if msg.angle3 < 90.0:
            angle3_down = - (msg.angle3 - 90)
            angle3_down_revert = angle3_down * 1 / 5
            angle3_turn = (-112.58 - angle3_down_revert) * rad
        else:
            angle3_turn = (-112.58 + ((msg.angle3 - 90) * 4 / 9)) * rad
        joint_q_map = [angle1_turn, angle2_turn, angle3_turn, -77.05 * rad, 90.66 * rad, 91.75 * rad]
        # print("angle1", msg.angle1)
        # print("angle2", msg.angle2)
        # print("angle3", msg.angle3)
        rtde_c.moveJ(joint_q_map, joint_q_speed, acceleration)

        if 10.0 < msg.wes_left < 80.0:
            gripper.close()
            # time.sleep(1.0)
            gripper.open()


def angle_subscriber():
    rospy.init_node('angle_subscriber', anonymous=True)
    rospy.Subscriber("/angle_info", angle, callback, queue_size=1, buff_size=100000)
    rospy.spin()


if __name__ == "__main__":
    angle_subscriber()
