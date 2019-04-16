#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Date: 2019/2/1
Author: Xu Yucheng
Abstract: Code for gender recognition
'''
import os
import cv2
import math
import rospy
import roslib
import base64
from aip import AipFace
from std_msgs.msg import Int8
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from kamerider_image_msgs.msg import GenderDetection
from kamerider_speech.msg import mission


class gender_recognition:
    def __init__(self):
        # 定义baidu ai 的APP_ID、API_KEY、SECRET_KEY
        """
        根据认证信息获取access_token.
        access_token的有效期为30天，切记需要每30天进行定期更换，或者每次请求都拉取新token；
        """
        self.APP_ID     = '15524162'
        self.API_KEY    = 'STRPajpcGiYazTadVsprWl56'
        self.SECRET_KEY = 'QK8G70TKjRl4WnqlGIAtIwgNGcV03p7X'
        
        #ros params
        self.result_gender = None
        self.target_gender = None
        self.take_photo_signal=False
        self.sub_control_topic_name=None
        self.sub_image_raw_topic_name=None
        self.pub_to_control_topic_name=None
        self.pub_gender_recognition_topic_name=None
        self.path_to_save_image=None
        self.pub_result=None
        self.speech_pub = None
        self.get_params()

    def get_params(self):
        self.sub_image_raw_topic_name          = rospy.get_param('sub_image_raw_topic_name',          '/astra/rgb/image_raw')
        self.sub_control_topic_name            = rospy.get_param('sub_control_topic_name',  '/control_to_image')
        self.pub_gender_recognition_topic_name = rospy.get_param('pub_gender_recognition_topic_name', '/kamerider_image/gender_recognition')
        self.pub_to_control_topic_name         = rospy.get_param('pub_to_control_topic_name',         '/image_to_control')
        self.path_to_save_image                = rospy.get_param('path_to_save_image',                '/home/nvidia/catkin_ws/src/kamerider_image/kamerider_image_detection/test_images/gender_image_capture.jpg')
        self.path_to_save_result               = rospy.get_param('path_to_save_result',               '/home/nvidia/catkin_ws/src/kamerider_image/kamerider_image_detection/result/gender_recognition_result.jpg')     
        #定义R发布器和订阅器，话题名通过ROS PARAM参数服务器获取
        rospy.Subscriber(self.sub_image_raw_topic_name, Image, self.imageCallback)
        rospy.Subscriber(self.sub_control_topic_name, mission, self.controlCallback)
        self.pub_result = rospy.Publisher(self.pub_gender_recognition_topic_name, GenderDetection, queue_size=1)
        self.speech_pub = rospy.Publisher("/kamerider_speech/input", String, queue_size=1)
        self.pub_control = rospy.Publisher(self.pub_to_control_topic_name, String, queue_size=1)

    def imageCallback(self, msg):
        if self.take_photo_signal:
            print ("[INFO] Start to take photo")
            bridge = CvBridge()
            self.take_photo_signal = False
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
                cv2.imwrite(self.path_to_save_image, cv_image)
            except CvBridgeError as e:
                print (e)
            self.detection()
    
    def image_encode_base64(self):
        with open(self.path_to_save_image, 'rb') as f:
            image = base64.b64encode(f.read())
        base64_image = str(image)
        return base64_image
        
    def controlCallback(self, msg):
        if msg.mission_type == 'gender':
            self.target_gender = msg.mission_name
            print ("[INFO] Signal Received")
            self.take_photo_signal = True


    #检测函数
    def detection(self):
        encode_image = self.image_encode_base64()
        imageType = 'BASE64'
        #初始化aipfacce对象
        aipface = AipFace(self.APP_ID, self.API_KEY, self.SECRET_KEY)
        #设置
        options = {
        'max_face_num': 10,
        'face_field': 'gender',
        "face_type": 'LIVE'
        }
        #接收返回的检测结果
        result = aipface.detect(encode_image, imageType, options)
        print (result)
        #初始化统计数据
        male_num=0
        female_num=0
        #定义OpenCV字体
        font = cv2.FONT_HERSHEY_SIMPLEX
        #读取原始图片
        cv_image = cv2.imread(self.path_to_save_image)
        face_num = result['result']['face_num']
        #在原图像上标注人脸，并统计男性和女性的数目
        for i in range(face_num):
            location = result['result']['face_list'][i]['location']
            left_top = (int(location['left']), int(location['top']))
            right_bottom = (int(left_top[0] + location['width']), int(left_top[1] + location['height']))
            print ("No.{} face position is: {}".format(i+1, location))
            if result['result']['face_list'][i]['gender']['type'] == 'male':
                cv2.rectangle(cv_image, left_top, right_bottom, (255, 0, 0), 2)
                cv2.putText(cv_image, 'male',(left_top[0]+30, left_top[1]+30), font, 1.2, (255, 0, 0), 2)
            if result['result']['face_list'][i]['gender']['type'] == 'female':
                cv2.rectangle(cv_image, left_top, right_bottom, (0, 0, 255), 2)
                cv2.putText(cv_image, 'female',(left_top[0]+30, left_top[1]+30), font, 1.2, (0, 0, 255), 2)
            if result['result']['face_list'][i]['gender']['type'] == 'male' :
                male_num=male_num+1
            if result['result']['face_list'][i]['gender']['type'] == 'female' :
                female_num = female_num + 1
        cv2.imwrite(self.path_to_save_result, cv_image)
        #生成自定义消息类型的对象
        gender_message = GenderDetection()
        gender_message.female_num = female_num
        gender_message.male_num   = male_num
        gender_message.sit_num    = 0
        gender_message.stand_num  = 0

        if self.target_gender == "none":
            msg = String()
	    control_msg = String()
            msg.data = "This is the result of gender recognition"
            self.speech_pub.publish(msg)
            rospy.sleep(3)
            msg.data = "male number is " + str(male_num)
            self.speech_pub.publish(msg)
            rospy.sleep(3)
            msg.data = "female number is "+ str(female_num)
            self.speech_pub.publish(msg)
            self.pub_result.publish(gender_message)
            control_msg.data = "target_gender_detected"
            self.pub_control.publish(control_msg)

        if self.target_gender != "none":
            msg = String()
            control_msg = String()
            if self.target_gender == "male" or self.target_gender == "boy" or self.target_gender == "man":
                if male_num != 0:
                    msg.data = "I have found the male person"
                    control_msg.data = "target_gender_detected"
                    self.pub_control.publish(control_msg)
                    self.speech_pub.publish(msg)
                if male_num == 0:
                    msg.data = "sorry i cannot find {} person i will try again".format(self.target_gender)
                    control_msg.data = "target_gender_not_detected"
                    self.pub_control.publish(control_msg)
                    self.speech_pub.publish(msg)
            if self.target_gender == "female" or self.target_gender == 'women' or self.target_gender == 'girl':
                if female_num != 0:
                    control_msg.data = "target_gender_detected"
                    msg.data = "I have found the female person"
                    self.pub_control.publish(control_msg)
                    self.speech_pub.publish(msg)
                if female_num == 0:
                    msg.data = "sorry i cannot find {} person i will try again".format(self.target_gender)
                    control_msg.data = "target_gender_not_detected"
                    self.pub_control.publish(control_msg)
                    self.speech_pub.publish(msg)
                
        # 保存并显示处理后的图片
        # cv2.imshow('result', cv2.imread(self.path_to_save_result))
        # cv2.waitKey(100)


if __name__ == '__main__':
    #初始化节点
    rospy.init_node('gender_recognition')
    print ('----------init----------')
    print ('-----WAITING FOR IMAGE-----')
    gender_recognition()
    rospy.spin()
