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
#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

ros::Subscriber img_sub;
ros::Subscriber ctrl_sub;
vector<cv::Mat> temp;

std::string DATASET_DIR = "/home/nvidia/catkin_ws/src/kamerider_image/kamerider_image_pcl/robocup_2019/JPEGImages/";

int pCount = 3089;
bool save_image = false;

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
    cv::waitKey(10);
    if(temp.size() < 100)
    {
        temp.push_back(img); 
    }
    else
    {
        temp.erase(temp.begin());
    }
}

void ctrlCallback (const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "start")
    {
        for (int i =0; i< temp.size(); i++)
        {
            string full_path = DATASET_DIR + int2str(pCount) + ".png";
            cv::imwrite(full_path, temp[i]); 
            pCount ++;   
        }
        
    }
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "take_photo");
    ros::NodeHandle nh;

    img_sub = nh.subscribe ("/astra/rgb/image_raw", 1, imageCallback);
    ctrl_sub = nh.subscribe ("/take_photo", 1, ctrlCallback);
    ros::spin();
    return 0 ;
}
