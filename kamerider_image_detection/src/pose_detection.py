#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Date: 2019/02/03
Author: Xu Yucheng
Abstract: pose detect code with openpose
'''
import os
import sys
import math
import cv2
import rospy
import roslib
import base64
import numpy as np
from std_msgs.msg import Int8
from std_msgs.msg import String 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# openpose module
from openpose import pyopenpose as op

IMAGE_DIR = "/home/nvidia/catkin_ws/src/kamerider_image/kamerider_image_detection/result/"

class pose_detection:
    def __init__(self):
        self.params = dict()
        self.start_pose_detect = False
        self.take_photo = False
        self.pose_detect_finish = False
        self.sub_image_raw_topic_name = None
        self.sub_control_back_topic_name = None
        self.pub_pose_detect_result_topic_name = None
        self.pub_to_control = None
        self.set_param()

        # start openpose
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(self.params)
        self.opWrapper.start()
    
    def set_param(self):
        # ROS param参数服务器中取得参数值
        self.sub_image_raw_topic_name             = rospy.get_param('sub_image_raw_topic_name',          '/astra/rgb/image_raw')
        self.sub_control_back_topic_name          = rospy.get_param('sub_control_back_topic_name',       '/found_person')
        self.pub_pose_detect_result_topic_name    = rospy.get_param('pub_pose_detect_result_topic_name', '/kamerider_image/pose_detect')
        self.pub_to_control                       = rospy.get_param("pub_to_control",                    '/kamerider_speech/input')
        
        # 发布器和订阅器
        rospy.Subscriber(self.sub_image_raw_topic_name, Image, self.imageCallback)
        rospy.Subscriber(self.sub_control_back_topic_name, String, self.ctrlCallback)
        self.pub_result = rospy.Publisher(self.pub_pose_detect_result_topic_name, String, queue_size=1)
        self.pub_control = rospy.Publisher(self.pub_to_control, String, queue_size=1)
        
        # 设置openpose手部特征点检测时的参数
        self.params["model_folder"] = "/home/nvidia/openpose/models"

    def ctrlCallback(self, msg):
        if msg.data == "found_person":
            rospy.loginfo("found person start to take a photo")
            self.take_photo = True
    
    def imageCallback(self, msg):
        if self.take_photo:
            bridge = CvBridge()
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.pose_detect(cv_image)
                self.take_photo = False
            except CvBridgeError as e:
                print (e)
    
    def pose_detect(self, cv_image):
        # Create new datum
        datum = op.Datum()
        datum.cvInputData = cv_image
        # Process and display image
        self.opWrapper.emplaceAndPop([datum])
        # Display Image
        print("Body keypoints: \n" + str(datum.poseKeypoints))
        print("Face keypoints: \n" + str(datum.faceKeypoints))
        cv2.imwrite(IMAGE_DIR + "pose_detect_result.png", datum.cvOutputData)
        self.parse_pose(datum.poseKeypoints[0])
    
    def parse_pose(self, pose_points):
        #     {0,  "Nose"},
        #     {1,  "Neck"},
        #     {2,  "RShoulder"},
        #     {3,  "RElbow"},
        #     {4,  "RWrist"},
        #     {5,  "LShoulder"},
        #     {6,  "LElbow"},
        #     {7,  "LWrist"},
        #     {8,  "MidHip"},
        #     {9,  "RHip"},
        #     {10, "RKnee"},
        #     {11, "RAnkle"},
        #     {12, "LHip"},
        #     {13, "LKnee"},
        #     {14, "LAnkle"},
        #     {15, "REye"},
        #     {16, "LEye"},
        #     {17, "REar"},
        #     {18, "LEar"},
        #     {19, "LBigToe"},
        #     {20, "LSmallToe"},
        #     {21, "LHeel"},
        #     {22, "RBigToe"},
        #     {23, "RSmallToe"},
        #     {24, "RHeel"},
        #     {25, "Background"}
            # 首先判断人是站着还是躺在地上（不脏么。。。）
            # 判断Neck和MidHip 之间y轴方向上的距离和x轴方向上的距离大小
            if (abs(pose_points[1][0] - pose_points[8][0]) > abs(pose_points[1][1] - pose_points[8][1])):
                rospy.loginfo("The person in front of me is lying on the ground")
                self.pub_control.publish("The person in front of me is lying on the ground")
                self.pub_result.publish("lying down")
                self.pose_detect_finish = True
            
            else:
                # 然后判断这个人是举左手还是举右手
                if (pose_points[6][1] < pose_points[5][1]) or (pose_points[7][1] < pose_points[6][1]):
                    rospy.loginfo("The person in front of me is raising his left hand")
                    self.pub_control.publish("The person in front of me is raising his left hand")
                    self.pub_result.publish("raising left hand")
                if (pose_points[3][1] < pose_points[2][1]) or (pose_points[4][1] < pose_points[3][1]):
                    rospy.loginfo("The person in front of me is raising his right hand")
                    self.pub_control.publish("The person in front of me is raising his right hand")
                    self.pub_result.publish("raising left hand")
                
                # 判断这个人是指向左边还是右边
                if (abs(pose_points[6][0] - pose_points[5][0]) > abs(pose_points[3][0] - pose_points[2][0])):
                    rospy.loginfo("The person in front of me is pointing to the left")
                    self.pub_control.publish("The person in front of me is pointing to the left")
                    self.pub_result.publish("pointing to the left")
                if (abs(pose_points[6][0] - pose_points[5][0]) < abs(pose_points[3][0] - pose_points[2][0])):
                    rospy.loginfo("The person in front of me is pointing to the right")
                    self.pub_control.publish("The person in front of me is pointing to the right")
                    self.pub_result.publish("pointing to the right")
    
if __name__ == '__main__':
    rospy.init_node("pose_detection")
    detector = pose_detection()
    rospy.spin()


	
