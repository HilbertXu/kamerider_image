/*
!!!!CODE FOR TEST
Date: 2019/1/31
Author: Xu Yucheng
Abstract: A pure dlib face detection demo
*/
// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// C++
#include <iostream>
#include <vector>
#include <stdlib.h>

// kamerider_image_msgs
#include "kamerider_image_detection/timestramp.h"
#include <kamerider_image_msgs/FaceDetection.h>
#include <kamerider_image_msgs/BoundingBox.h>

int main(int argc, char** argv)
{
    kamerider_image_msgs::FaceDetection msg;
    msg.face_num=2;
    kamerider_image_msgs::BoundingBox bbox;
    std::vector<kamerider_image_msgs::BoundingBox> bounding_boxes;
    
    for (int i=0; i < 2; i++)
    {
        bbox.Class = "face";
        bbox.xmin = i*100;
        bbox.xmax = (i+1)*100;
        bbox.ymin = (i*200);
        bbox.ymax = (i+1)*200;
        bounding_boxes.push_back(bbox);
    }
    msg.bounding_boxes = bounding_boxes;

    ros::init(argc, argv, "msg_test");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<kamerider_image_msgs::FaceDetection>("/talking", 1);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
    
}