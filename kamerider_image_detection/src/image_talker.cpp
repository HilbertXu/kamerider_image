/*
!!!!CODE FOR TEST USEs
Date: 2019/1/31
Author: Xu Yucheng
Abstract: Read a image and send it as ros message
*/
// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// C++
#include <iostream>
#include <vector>
#include <stdlib.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    ros::init (argc, argv, "image_talker");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/rgb/image_raw", 1);

    cv::Mat image = cv::imread("/home/kamerider/Pictures/right.png", CV_LOAD_IMAGE_COLOR);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    ros::Rate loop_rate(5);
    while (nh.ok())
    {
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}




