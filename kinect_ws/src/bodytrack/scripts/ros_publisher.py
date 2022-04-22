#!/usr/bin/env python
# -*- coding: utf-8 -*-
# bodytrack, generate vedio and publish angle to robotarm

import sys

# sys.path.insert(1, '/utils/')

import numpy as np
from pyKinectAzure import pyKinectAzure, _k4a
from kinectBodyTracker import kinectBodyTracker, _k4abt
import cv2
import math
from PIL import Image, ImageDraw, ImageFont

import rospy
from bodytrack.msg import angle

# Path to the module
# TODO: Modify with the path containing the k4a.dll from the Azure Kinect SDK
modulePath = '/usr/lib/x86_64-linux-gnu/libk4a.so'
bodyTrackingModulePath = '/usr/lib/libk4abt.so'


def calculate_3D_angle(elbow_x, elbow_y, elbow_z, shoulder_x, shoulder_y, shoulder_z, wrist_x, wrist_y, wrist_z):
    # vectors of elbow to shoulder and elbow to wrist
    etsx, etsy, etsz = (elbow_x - shoulder_x), (elbow_y - shoulder_y), (elbow_z - shoulder_z)

    etwx, etwy, etwz = (elbow_x - wrist_x), (elbow_y - wrist_y), (elbow_z - wrist_z)

    # calculate angle
    cos_wes = (etsx * etwx + etsy * etwy + etsz * etwz) / (
            (math.sqrt(etsx ** 2 + etsy ** 2 + etsz ** 2)) *
            (math.sqrt(etwx ** 2 + etwy ** 2 + etwz ** 2)))  # angle cosine
    angle = math.degrees(math.acos(cos_wes))  # angle
    return angle


def calculate_2D_angle(v1, v2):
    # product of two vectors norm
    TheNorm = np.linalg.norm(v1) * np.linalg.norm(v2)
    rho = np.rad2deg(np.arcsin(np.cross(v1, v2) / TheNorm))
    theta = np.rad2deg(np.arccos(np.dot(v1, v2) / TheNorm))
    if rho < 0:
        return -theta
    else:
        return theta


# show chinese on image
def cv2ImgAddText(img, text, left, top, textColor=(0, 255, 0), textSize=20):
    if isinstance(img, np.ndarray): 
        img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img)
    fontText = ImageFont.truetype('NotoSansCJK-Bold.ttc', textSize, encoding="utf-8")
    draw.text((left, top), text, textColor, font=fontText)
    return cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)


def angle_publisher():
    # ros publish
    rospy.init_node('angle_publisher', anonymous=True)
    angle_info_pub = rospy.Publisher('/angle_info', angle, queue_size=1)
    pyK4A = pyKinectAzure(modulePath)

    # Open device
    pyK4A.device_open()

    # Modify camera configuration
    device_config = pyK4A.config
    device_config.color_resolution = _k4a.K4A_COLOR_RESOLUTION_720P
    device_config.depth_mode = _k4a.K4A_DEPTH_MODE_WFOV_2X2BINNED
    # print(device_config)

    # Start cameras using modified configuration
    pyK4A.device_start_cameras(device_config)

    # Initialize the body tracker
    pyK4A.bodyTracker_start(bodyTrackingModulePath)

    k = 0
    # control chinese message under different condition
    showFlag = False

    while not rospy.is_shutdown():
        # Get capture
        pyK4A.device_get_capture()

        # Get the depth image from the capture
        depth_image_handle = pyK4A.capture_get_depth_image()

        # Get the color image from the capture
        color_image_handle = pyK4A.capture_get_color_image()

        # Check the image has been read correctly
        if color_image_handle:

            # Perform body detection
            pyK4A.bodyTracker_update()

            # Read and convert the image data to numpy array:
            color_image = pyK4A.image_convert_to_numpy(color_image_handle)

            # Read and convert the image data to numpy array:
            depth_image = pyK4A.image_convert_to_numpy(depth_image_handle)
            depth_color_image = cv2.convertScaleAbs(depth_image,
                                                    alpha=0.05)  # alpha is fitted by visual comparison with Azure
            # k4aviewer results
            depth_color_image = cv2.cvtColor(depth_color_image, cv2.COLOR_GRAY2RGB)

            # Get body segmentation image
            body_image_color = pyK4A.bodyTracker_get_body_segmentation()

            # 用于判断是否检测到人体
            if len(pyK4A.body_tracker.bodiesNow) == 0:
                stopFlag = 0
            else:
                stopFlag = 1

            if stopFlag == 0:
                showFlag = False

            angle_msg = angle()
            angle_msg.stoptrack = showFlag

            if stopFlag == 0:
                color_image = cv2ImgAddText(color_image, '等待人进入', 50, 50, (85, 170, 119), 80)

            # Draw the skeleton
            for body in pyK4A.body_tracker.bodiesNow:
                skeleton2D = pyK4A.bodyTracker_project_skeleton(body.skeleton, _k4a.K4A_CALIBRATION_TYPE_COLOR)
                color_image = pyK4A.body_tracker.draw2DSkeleton(skeleton2D, body.id, color_image)
                spine_naval_x = pyK4A.body_tracker.get_body_skeleton().joints[1].position.xyz.x
                spine_naval_y = pyK4A.body_tracker.get_body_skeleton().joints[1].position.xyz.y
                spine_naval_z = pyK4A.body_tracker.get_body_skeleton().joints[1].position.xyz.z
                spine_chest_x = pyK4A.body_tracker.get_body_skeleton().joints[2].position.xyz.x
                spine_chest_y = pyK4A.body_tracker.get_body_skeleton().joints[2].position.xyz.y
                spine_chest_z = pyK4A.body_tracker.get_body_skeleton().joints[2].position.xyz.z
                neck_x = pyK4A.body_tracker.get_body_skeleton().joints[3].position.xyz.x
                neck_y = pyK4A.body_tracker.get_body_skeleton().joints[3].position.xyz.y
                neck_z = pyK4A.body_tracker.get_body_skeleton().joints[3].position.xyz.z
                shoulder_left_x = pyK4A.body_tracker.get_body_skeleton().joints[5].position.xyz.x
                shoulder_left_y = pyK4A.body_tracker.get_body_skeleton().joints[5].position.xyz.y
                shoulder_left_z = pyK4A.body_tracker.get_body_skeleton().joints[5].position.xyz.z
                elbow_left_x = pyK4A.body_tracker.get_body_skeleton().joints[6].position.xyz.x
                elbow_left_y = pyK4A.body_tracker.get_body_skeleton().joints[6].position.xyz.y
                elbow_left_z = pyK4A.body_tracker.get_body_skeleton().joints[6].position.xyz.z
                wrist_left_x = pyK4A.body_tracker.get_body_skeleton().joints[7].position.xyz.x
                wrist_left_y = pyK4A.body_tracker.get_body_skeleton().joints[7].position.xyz.y
                wrist_left_z = pyK4A.body_tracker.get_body_skeleton().joints[7].position.xyz.z
                shoulder_right_x = pyK4A.body_tracker.get_body_skeleton().joints[12].position.xyz.x
                shoulder_right_y = pyK4A.body_tracker.get_body_skeleton().joints[12].position.xyz.y
                shoulder_right_z = pyK4A.body_tracker.get_body_skeleton().joints[12].position.xyz.z
                elbow_right_x = pyK4A.body_tracker.get_body_skeleton().joints[13].position.xyz.x
                elbow_right_y = pyK4A.body_tracker.get_body_skeleton().joints[13].position.xyz.y
                elbow_right_z = pyK4A.body_tracker.get_body_skeleton().joints[13].position.xyz.z
                wrist_right_x = pyK4A.body_tracker.get_body_skeleton().joints[14].position.xyz.x
                wrist_right_y = pyK4A.body_tracker.get_body_skeleton().joints[14].position.xyz.y
                wrist_right_z = pyK4A.body_tracker.get_body_skeleton().joints[14].position.xyz.z

                # 三个轴方向上的单位向量
                x_unit = np.array([1, 0, 0])
                y_uint = np.array([0, 1, 0])
                z_unit = np.array([0, 0, 1])

                # 计算左右挥手角度对应机械臂底部自由度
                # 底部自由度，左右挥手角度对应机械臂底部自由度
                vector_neck_shoulder_right = np.array(
                    [neck_x - shoulder_right_x, neck_y - shoulder_right_y, neck_z - shoulder_right_z])
                vector_neck_elbow_right = np.array(
                    [neck_x - elbow_right_x, neck_y - elbow_right_y, neck_z - elbow_right_z])

                # 计算neck_elbow_right在y轴上的投影
                projy_vector_ne = np.array(vector_neck_elbow_right).dot(y_uint) * y_uint

                # 计算neck_elbow_right在xz平面上的投影
                projxz_vector_ne = vector_neck_elbow_right - projy_vector_ne

                # 计算上臂在xz平面上的投影
                proj_upper_hand = projxz_vector_ne - vector_neck_shoulder_right

                # 计算proj_upper_hand 的长度
                L_proj_upper_hand = np.sqrt(proj_upper_hand.dot(proj_upper_hand))

                # 计算angle1
                angle_initial = (math.degrees(
                    np.arccos(proj_upper_hand.dot(z_unit) / L_proj_upper_hand)))
                if elbow_right_x - neck_x < 0:
                    angle1 = angle_initial
                else:
                    angle1 = - angle_initial
                # print("--------------angle1------------------", angle1)

                # 计算上臂与两肩之间的夹角对应机械臂自下而上第二个自由度
                # 计算vector_e_n在z轴上的投影
                projz_vector_ne = vector_neck_elbow_right.dot(z_unit) * z_unit

                # 计算vector_neck_elbow在xy平面上的投影
                projxy_vector_ne = vector_neck_elbow_right - projz_vector_ne

                # 计算上臂在xy平面上的投影
                projxy_upper_hand = projxy_vector_ne - vector_neck_shoulder_right

                # 计算projxy_upper_hand的长度
                L_projxy_upper_hand = np.sqrt(projxy_upper_hand.dot(projxy_upper_hand))

                # 计算angle2
                angle2 = (math.degrees(
                    np.arccos(projxy_upper_hand.dot(y_uint) / L_projxy_upper_hand)))
                angle2 = - (angle2 - 180)

                # the angle between elbow to should and elbow to wrist
                angle_wes = calculate_3D_angle(elbow_right_x, elbow_right_y, elbow_right_z, shoulder_right_x,
                                               shoulder_right_y, shoulder_right_z, wrist_right_x, wrist_right_y,
                                               wrist_right_z)
                # print("angle_wes", angle_wes)
                angle3 = angle_wes
                # print("--------------angle3------------------", angle3)

                angle_wes_left = calculate_3D_angle(elbow_left_x, elbow_left_y, elbow_left_z, shoulder_left_x,
                                                    shoulder_left_y, shoulder_left_z, wrist_left_x, wrist_left_y,
                                                    wrist_left_z)
                # print("------------angle_wes_lef--------------", angle_wes_left)

                # angle between hand-elbow and spine map to robot arm bottom degree
                v_spine = [spine_naval_x - spine_chest_x, spine_naval_y - spine_chest_y]
                v_hand_elbow = [elbow_right_x - wrist_right_x, elbow_right_y - wrist_right_y]
                angle_turn = calculate_2D_angle(v_spine, v_hand_elbow)

                # angle_msg = angle()
                angle_msg.wes = angle_wes
                angle_msg.wes_left = angle_wes_left
                angle_msg.turn = angle_turn
                angle_msg.angle1 = angle1
                angle_msg.angle2 = angle2
                angle_msg.angle3 = angle3

                # 用于判断启动人机交互
                if 70.0 < angle_wes < 110.0 and -10 < angle_turn < 10:
                    showFlag = True

                if not showFlag:
                    color_image = cv2ImgAddText(color_image, '举起右手开始交互', 50, 50, (0, 255, 102), 70)

                if showFlag:
                    color_image = cv2ImgAddText(color_image, '人机交互中', 900, 50, (0, 51, 255), 60)

            cv2.namedWindow('Color Skeleton Image', cv2.WINDOW_NORMAL)
            cv2.setWindowProperty('Color Skeleton Image', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.imshow('Color Skeleton Image', color_image)
            k = cv2.waitKey(1)
            angle_info_pub.publish(angle_msg)
        if k == 27:  # Esc key to stop
            break
        elif k == ord('q'):
            cv2.imwrite('outputImage.jpg', color_image)


if __name__ == "__main__":
    try:
        angle_publisher()
    except rospy.ROSInterruptException:
        pass
