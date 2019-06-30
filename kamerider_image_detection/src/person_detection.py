#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Date: 2019/2/1
Author: Xu Yucheng
Abstract: Code for person detection
'''
import os
import cv2
import math
import rospy
import roslib
import base64
from aip import AipBodyAnalysis
from std_msgs.msg import Int8
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from kamerider_image_msgs.msg import mission



class person_detection:
    def __init__(self):
        # 定义baidu ai 的APP_ID、API_KEY、SECRET_KEY
        """
        根据认证信息获取access_token.
        access_token的有效期为30天，切记需要每30天进行定期更换，或者每次请求都拉取新token；
        """
        self.APP_ID     = '16002184'
        self.API_KEY    = 'W6jgTXcHFZioPcmXWilfaGO2'
        self.SECRET_KEY = 'oNewRv2mtqfknwzq3rgEFc3sP4SAbI0v'
        
        #ros params
        self.msg_count = 0
        self._turn = False
        self.find_person = True
        self.start_detect = False
        self.angle_count = 0
        self.take_photo_signal = False
        self.sub_image_raw_topic_name=None
        self.sub_control_topic_name = None
        self.pub_person_detect_topic_name=None

        self.path_to_save_image=None
        self.pub_result=None
        self.move_pub = None
        self.speech_pub = None
        self.client = None
        self.get_params()
        self.turn_robot(1.8)
        while (True):
            if self.find_person == False:
                rospy.loginfo("Msg received start person detection")
                if self.start_detect == True:
                    self.detection()

    def get_params(self):
        self.sub_image_raw_topic_name          = rospy.get_param('sub_image_raw_topic_name',          '/astra/rgb/image_raw')
        self.sub_control_topic_name            = rospy.get_param("sub_control_topic_name",            '/control_to_image')
        self.pub_person_detect_topic_name      = rospy.get_param('pub_person_detect_topic_name',      '/image_to_control')
        self.pub_to_move_base_topic_name       = rospy.get_param('pub_to_move_base_topic_name',       '/cmd_vel_mux/input/navi')
        self.pub_to_speech_topic_name          = rospy.get_param("pub_to_speech_topic_name",          '/kamerider_speech/input')
        self.path_to_save_image                = rospy.get_param('path_to_save_image',                '/home/nvidia/catkin_ws/src/kamerider_image/kamerider_image_detection/result/person_image_capture.jpg')
        self.path_to_save_result               = rospy.get_param('path_to_save_result',               '/home/nvidia/catkin_ws/src/kamerider_image/kamerider_image_detection/result/person_detection_result.jpg')     
        #定义R发布器和订阅器，话题名通过ROS PARAM参数服务器获取
        rospy.Subscriber(self.sub_image_raw_topic_name, Image, self.imageCallback)
        rospy.Subscriber(self.sub_control_topic_name, mission, self.controlCallback)
        self.pub_result = rospy.Publisher(self.pub_person_detect_topic_name, String, queue_size=1)
        self.move_pub   = rospy.Publisher(self.pub_to_move_base_topic_name, Twist, queue_size=1)
        self.speech_pub = rospy.Publisher(self.pub_to_speech_topic_name, String, queue_size=1)
    
    def controlCallback(self, msg):
        if msg.mission_type == "person":
            self.msg_count += 1
            if self.msg_count > 1:
                self.forward_robot(-0.5)
                self.turn_robot(1.8)
            self.find_person = False
            self.take_photo_signal = True

    def imageCallback(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        #cv2.imshow("current", cv_image)
        #cv2.waitKey(10)
        if self.take_photo_signal == True:
            print ("[INFO] Start to take photo")
            try:
                cv2.imwrite(self.path_to_save_image, cv_image)
                cv2.imshow("frame", cv_image)
                cv2.waitKey(100)
            except CvBridgeError as e:
                print (e)
            self.start_detect = True
            self.take_photo_signal = False
            
    def image_encode(self):
        with open(self.path_to_save_image, 'rb') as f:
            return f.read()

    #检测函数
    def detection(self):
        self.start_detect = False
        encode_image = self.image_encode()
        print (type(encode_image))
        image = cv2.imread(self.path_to_save_image)
        #初始化aipfacce对象
        client = AipBodyAnalysis(self.APP_ID, self.API_KEY, self.SECRET_KEY)
        #接收返回的检测结果
        result = client.bodyAttr(encode_image)
        print (result)
        num = result["person_num"]
        if result["person_num"] == 0:
            rospy.loginfo("Start Turning robot")
            self.turn_robot(1.5)
            self.angle_count += 1.5
            self.take_photo_signal = True

        if result["person_num"] > 1:
            self.speech_pub.publish("I can see more than one person, I have found the target person")
            self.speech_pub.publish("Please stand in front of me and lead me")
            for i in range(num):
                person_info = result["person_info"][i]
                point1 = (person_info["location"]["left"], person_info["location"]["top"])
                point2 = (person_info["location"]["left"]+person_info["location"]["width"], person_info["location"]["top"]+person_info["location"]["height"])
                cv2.rectangle(image, point1, point2, (0,0,255), 2)
            cv2.imwrite(self.path_to_save_result, image)
            self.find_person = True
            msg = String()
            msg.data = "person_target_found"
            self.pub_result.publish(msg)

        if result["person_num"] == 1:
            person_info = result["person_info"][0]
            person_x = person_info["location"]["left"] + person_info["location"]["width"]/2
            person_y = person_info["location"]["top"] + person_info["location"]["height"]/2
            point1 = (person_info["location"]["left"], person_info["location"]["top"])
            point2 = (person_info["location"]["left"]+person_info["location"]["width"], person_info["location"]["top"]+person_info["location"]["height"])
            
            cv2.rectangle(image, point1, point2, (0,0,255), 2)
            cv2.imwrite(self.path_to_save_result, image)
            
            if person_x-320 > 80:
                self.turn_robot(-1.1)
            if person_x -320 < -80:
                self.turn_robot(1.1)
            if (person_info["location"]["top"] < 240):
                self.forward_robot(0.5)
                self.find_person = True
                rospy.loginfo("Person Target Reached!")
                control_msg = String()
                speech_msg = String()
                control_msg.data = "person_target_found"
                speech_msg.data = "I have reached the target person"
                self.speech_pub.publish(speech_msg)
                self.pub_result.publish(control_msg) 
        
            
    def turn_robot(self, angle):
        vel = Twist()
        count = 0
        time = 2
        rate = 10
        r = rospy.Rate(rate)
        num = time*10
        theta = angle/time
        vel.angular.z = theta
        while (count < num):
            count += 1
            self.move_pub.publish(vel)
            r.sleep()
        vel.angular.z = 0.0
        self.move_pub.publish(vel)
        print ("Turn robot~")
    
    def forward_robot(self, dis):
        vel = Twist()
        count = 0
        time = 4
        vel.linear.x = dis/time
        rate = 10
        r = rospy.Rate(rate)
        num = time*10
        while (count < num):
            count += 1
            self.move_pub.publish(vel)
            r.sleep()
        
        vel.linear.x = 0.0
        self.move_pub.publish(vel)
        print ("Go forward~")
        

if __name__ == '__main__':
    #初始化节点
    rospy.init_node('person_detection')
    print ('----------init----------')
    print ('-----WAITING FOR IMAGE-----')
    detector = person_detection()
    rospy.spin()
        


