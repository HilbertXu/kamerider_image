/*
Date: 2018/12/16
Author: Xu Yucheng
一个简单的采集数据集的程序
*/
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

ros::Subscriber img_sub;

std::string DATASET_DIR = "/home/kamerider/darknet/RoboCup_2019/dataset/JPEGImages/";

int pCount = 3;

string int2str(int &int_temp)
{
    stringstream stream;
    stream<<int_temp;
    string string_temp = stream.str();

    return string_temp;
}

void imageCallback (const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr -> image;
    cv::namedWindow("demo", CV_WINDOW_AUTOSIZE);
    cv::imshow("demo", img);
    int key = cv::waitKey(10);
    if (key == 's')
    {
        for (int i=0; i<50; i++)
        {
            int num = i + pCount*50;
            std::string idx = int2str(num);
            std::string FULL_PATH = DATASET_DIR + idx + ".jpg";
            ROS_INFO ("Writing RGB image %s", FULL_PATH.c_str());
            cv::imwrite (FULL_PATH, img);
        }
        pCount ++;
    }
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "take_photo");
    ros::NodeHandle nh;

    img_sub = nh.subscribe ("/image_raw", 1, imageCallback);

    ros::spin();
    return 0 ;
}
