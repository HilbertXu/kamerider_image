#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Date: 2019/2/3
Author: Xu Yucheng
Abstract: Code for face detection in ros by using Dlib(python)
'''
import os
import cv2
import sys
import math
import dlib
import rospy
import roslib
import timeit
import base64
import numpy as np
from std_msgs.msg import Int8
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from kamerider_image_msgs.msg import BoundingBox
from kamerider_image_msgs.msg import FaceDetection

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
class face_detection:
    @clock
    def __init__(self):
        self.keypoint_detect               = None
        #将dlib识别出来的关键点的shape转换成numpy矩阵的格式，方便之后进行调用
        self.shape_np                      = None
        self.sub_image_raw_topic_name      = None
        self.pub_face_detection_topic_name = None
        self.path_to_pretrained_dataset    = None
        self.path_to_save_image            = None
        self.pub_result                    = None
        #实例化检测器
        self.detector                      = dlib.get_frontal_face_detector()
        self.message                       = FaceDetection()
        self.get_params()
    
    @clock
    def get_params(self):
        #ROS param参数服务器中取得参数值
        self.keypoint_detect               = rospy.get_param('keypoint_detect',               False)
        self.sub_image_raw_topic_name      = rospy.get_param('sub_image_raw_topic_name',      '/camera/rgb/image_raw')
        self.pub_face_detection_topic_name = rospy.get_param('pub_face_detection_topic_name', '/kamerider_image/face_detection')
        self.path_to_pretrained_dataset    = rospy.get_param('path_to_pretrained_dataset',    '/home/kamerider/catkin_ws/src/kamerider_image/kamerider_image_detection/dataset/shape_predictor_68_face_landmarks.dat')
        self.path_to_save_image            = rospy.get_param('path_to_save_image',            '/home/kamerider/catkin_ws/src/kamerider_image/kamerider_image_detection/result/face_detection_result.png')
        
        #发布器和订阅器
        rospy.Subscriber(self.sub_image_raw_topic_name, Image, self.imageCallback)
        self.pub_result = rospy.Publisher(self.pub_face_detection_topic_name, FaceDetection, queue_size=1)
    
    def shape_to_np(self, shape, dtype='int'):
        coords = np.zeros((68,2), dtype=dtype)
        for i in range(68):
            coords[i] = (shape.part(i).x, shape.part(i).y)
        self.shape_np = coords
    
    def rect_to_bbox(self, rect):
        x_min = rect.left()
        y_min = rect.top()
        x_max = rect.right()
        y_max = rect.bottom()
        return (x_min, y_min, x_max, y_max)

    def imageCallback(self, msg):
        print ("[INFO] Receiving image")
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print (e)
        self.detection(cv_image)
    
    def publishMessage(self, rects):
        self.message.face_num = len(rects)
        self.message.bounding_boxes=[]
        temp_bbox = BoundingBox()
        for (i, rect) in enumerate(rects):
            rect_coords = self.rect_to_bbox(rect)
            temp_bbox.Class = 'face'
            temp_bbox.xmin = rect_coords[0]
            temp_bbox.ymin = rect_coords[1]
            temp_bbox.xmax = rect_coords[2]
            temp_bbox.ymax = rect_coords[3]
            self.message.bounding_boxes.append(temp_bbox)
        self.pub_result.publish(self.message)
    '''
    @TODO
    需要测试在实际情况下对人脸应该进行几次上采样，才能获得更好地识别效果
    修改rects = detector(gray_image, 2)第二个参数
    在测试图片中，一次上采样会有比较小的人脸无法识别的情况
    两次上采样可以很好地解决这个问题，但是相比之下识别速度会减慢很多,会有非常大的延迟
    '''
    #@clock              
    def detection(self, cv_image):
        cv_image = cv2.resize(cv_image, (640, 480))
        #图片灰度化
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #利用检测器检测人脸
        rects = self.detector(gray_image, 1)
        print ("The Number of face detected is: {}".format(len(rects)))
        #检测是否需要进行关键点检测
        if self.keypoint_detect:
            #实例化人脸特征点检测器
            predictor = dlib.shape_predictor(self.path_to_pretrained_dataset)
            for (i, rect) in enumerate(rects):
                shape = predictor(gray_image, rect)
                self.shape_to_np(shape)
                bbox = self.rect_to_bbox(rect)
                cv_image = cv2.rectangle(
                    cv_image, (bbox[0],bbox[1]), 
                    (bbox[2], bbox[3]), (0,0,255)
                    )
                for j in range(68):
                    cv2.circle(
                        cv_image, (shape.part(j).x, shape.part(j).y),
                        1, (0,0,255), thickness=2
                        )
            cv2.imwrite(self.path_to_save_image, cv_image)
            cv2.imshow("face_detection", cv_image)
            cv2.waitKey(1)
        
        else:
            for (i, rect) in enumerate(rects):
                bbox = self.rect_to_bbox(rect)
                cv_image = cv2.rectangle(
                    cv_image, (bbox[0],bbox[1]), 
                    (bbox[2], bbox[3]), (0,0,255)
                    )
            cv2.imwrite(self.path_to_save_image, cv_image)
            cv2.imshow("face_detection", cv_image)
            cv2.waitKey(1)
        self.publishMessage(rects)

if __name__ == '__main__':
    rospy.init_node('face_detection')
    print ('----------init----------')
    print ('-----WAITING FOR IMAGE-----')
    face_detection()
    rospy.spin()


            

    


