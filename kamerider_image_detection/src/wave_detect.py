#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Date: 2019/02/03
Author: Xu Yucheng
Abstract: Waving detect code with dlib and openposeS
'''
import os
import cv2
import sys
import math
import dlib
import rospy
import roslib
import base64
import numpy as np
from std_msgs.msg import Int8
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from kamerider_image_msgs.msg import BoundingBox
from kamerider_image_msgs.msg import FaceDetection
#openpose part
from openpose import pyopenpose as op

class wave_detect:
    def __init__(self):
        #params for openpose wrapper
        self.params = dict()
        self.find_waving_person            = False
        self.sub_image_raw_topic_name      = None
        self.pub_wave_detect_topic_name    = None
        self.pub_turn_robot_command        = None
        self.path_to_save_image            = None
        self.pub_result                    = None
        #在没有找到挥手的人的时候，需要转动机器人(测试时暂时不需要)
        #留出来接口，在launch文件中修改对应话题名称
        self.pub_turn_robot                = None
        self.get_params()

        # Starting OpenPose
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(self.params)
        self.opWrapper.start()
        
    def get_params(self):
        #ROS param参数服务器中取得参数值
        self.sub_image_raw_topic_name      = rospy.get_param('sub_image_raw_topic_name',      '/image_raw')
        self.pub_wave_detect_topic_name    = rospy.get_param('pub_wave_detect_topic_name',    '/kamerider_image/wave_detect')
        self.path_to_save_image            = rospy.get_param('path_to_save_image',            '/home/kamerider/catkin_ws/src/kamerider_image/kamerider_image_detection/result/wave_detect_result.png')
        self.pub_turn_robot_command        = rospy.get_param('pub_turn_robot_command',        '/kamerider_navigation/turn_robot_server')
        #发布器和订阅器
        rospy.Subscriber(self.sub_image_raw_topic_name, Image, self.imageCallback)
        self.pub_result = rospy.Publisher(self.pub_wave_detect_topic_name, FaceDetection, queue_size=1)
        #设置openpose手部特征点检测时的参数
        self.params["model_folder"] = "/home/kamerider/openpose/models"
        self.params["hand"] = True
        self.params["hand_detector"] = 2
        self.params["body_disable"] = True

    def rect_to_bbox(self, rect):
        '''
        (xmin, ymin)---------
        |                   |
        |                   |
        |                   |
        |---------(xmax, ymax)
        '''
        x_min = rect.left()
        y_min = rect.top()
        x_max = rect.right()
        y_max = rect.bottom()
        return (x_min, y_min, x_max, y_max)
    
    def face_region_zoom_out(self, rect):
        rect_coords = self.rect_to_bbox(rect)
        xmin = rect_coords[0]-100
        ymin = rect_coords[1]-50
        xmax = rect_coords[0]
        ymax = rect_coords[3]+50
        handRectangle = [
            op.Rectangle(rect_coords[0]-100, rect_coords[1]-50, 150, 150),
            op.Rectangle(rect_coords[2], rect_coords[1]-50, 150, 150)
        ]
        return handRectangle

    def imageCallback(self, msg):
        print ("[INFO] Receiving image")
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print (e)
        self.face_detect(cv_image)
    
    def face_detect(self, cv_image):
        cv_image = cv2.resize(cv_image, (640, 480))
        #图片灰度化
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #实例化检测器
        detector  = dlib.get_frontal_face_detector()
        #利用检测器检测人脸
        rects = detector(gray_image, 2)
        print ("The Number of face detected is: {}".format(len(rects)))
        handRectangles=[]
        #获得人脸在图像中的位置之后，将人脸两侧区域放大，然后在这个放大了的区域内检测手部
        for rect in rects:
            handRegion = self.face_region_zoom_out(rect)
            handRectangles.append(handRegion)
        self.openpose_hand_detect(handRectangles, cv_image)
    
    def openpose_hand_detect(self, handRectangles, cv_image):
        # Create new datum
        datum = op.Datum()
        datum.cvInputData = cv_image
        datum.handRectangles = handRectangles

        # Process and display image
        self.opWrapper.emplaceAndPop([datum])
        print("Left hand keypoints: \n" + str(datum.handKeypoints[0]))
        print("Right hand keypoints: \n" + str(datum.handKeypoints[1]))
        cv2.imshow("wave detect result", datum.cvOutputData)
        cv2.waitKey(100)

if __name__ == '__main__':
    rospy.init_node('wave_detect')
    print ('----------init----------')
    print ('-----WAITING FOR IMAGE-----')
    try:
        wave_detect()
    except:
        print ("Please check file paths in __init__ function")
    rospy.spin()



