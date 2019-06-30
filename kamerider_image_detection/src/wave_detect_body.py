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

#一个写成装饰器形式的计时器，在函数前加@clock便可以输出每次调用该函数运行时间
def clock(func):
    def clocked(*args):
        t0 = timeit.default_timer()
        result = func(*args)
        elapsed = timeit.default_timer() - t0
        name = func.__name__
        arg_str = ', '.join(repr(arg) for arg in args)
        print('[%0.8fs] %s(%s) -> %r' % (elapsed, name, arg_str, result))
        return result
    return clocked

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
        self.sub_image_raw_topic_name      = rospy.get_param('sub_image_raw_topic_name',      '/astra/rgb/image_raw')
        self.pub_wave_detect_topic_name    = rospy.get_param('pub_wave_detect_topic_name',    '/kamerider_image/wave_detect')
        self.path_to_save_image            = rospy.get_param('path_to_save_image',            '/home/nvidia/catkin_ws/src/kamerider_image/kamerider_image_detection/result/wave_detect_result.png')
        self.pub_turn_robot_command        = rospy.get_param('pub_turn_robot_command',        '/kamerider_navigation/turn_robot_server')
        #发布器和订阅器
        rospy.Subscriber(self.sub_image_raw_topic_name, Image, self.imageCallback)
        self.pub_result = rospy.Publisher(self.pub_wave_detect_topic_name, String, queue_size=1)
        #设置openpose手部特征点检测时的参数
        self.params["model_folder"] = "/home/nvidia/openpose/models"
        self.params["body"] = True
	self.params["hand_net_resolution"] = "96x96"
        self.params["hand_detector"] = 2
	self.params["body"] = 0

    def imageCallback(self, msg):
        print ("[INFO] Receiving image")
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print (e)
        self.face_detect(cv_image)
    
    def openpose_hand_detect(self, handRectangles, cv_image):
        # Create new datum
        datum = op.Datum()
        datum.cvInputData = cv_image
        datum.handRectangles = handRectangles

        # Process and display image
        self.opWrapper.emplaceAndPop([datum])
        # Uncomment to see the output of openpose
        # [hand_position_pixel_x, hand_position_pixel_y, probability]
        # print("Right hand keypoints: \n" + str(datum.handKeypoints[0]))
        # print("Left hand keypoints: \n" + str(datum.handKeypoints[1]))
        pCount_left = 0
        pCount_right = 0
        for i in range(2):
            # i=0 right hand
            # i=1 left hand
            temp_hand = datum.handKeypoints[i]
            for point in datum.handKeypoints[i][0]:
                if point[2] > 4e-1 and i == 0:
                    pCount_right += 1
                if point[2] > 4e-1 and i == 1:
                    pCount_left += 1
        if pCount_left > 6:
            self.pub_result.publish("Left hand detected")
        if pCount_right > 6:
            self.pub_result.publish("Right hand detected")
        if pCount_left < 6 and pCount_right < 6:
            self.pub_result.publish("No hand detected")
            
        cv2.imshow("wave detect result", datum.cvOutputData)
        cv2.imwrite("/home/nvidia/catkin_ws/src/kamerider_image/kamerider_image_detection/result/wave_detect_result.png", datum.cvOutputData)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('wave_detect')
    print ('----------init----------')
    print ('-----WAITING FOR IMAGE-----')
    try:
        wave_detect()
    except:
        print ("Please check file paths in __init__ function")
    rospy.spin()



