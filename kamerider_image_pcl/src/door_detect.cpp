/*
Date： 2018/12/12
Author: Xu Yucheng
使用PCL来检测机器人面前的门是否打开，检测当前画面中点云的数目
Astra PRO大概在离门34cm之内检测到点云数目为0
若门打开可以判断100帧内
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
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/thread/thread.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
//体素网格采样
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <unistd.h>

//namespace
using namespace std;
using namespace cv;
using namespace pcl;

//定义ros的发布器与订阅器
/*

设想场景是机器人开始任务时需要进门，在接受到“start“消息之后，检测门是否已经打开
如果门是关闭状态则通过语音说出“Please help me open the door”
以及："if the door is opened please say 'Jack the door is open'"
语音识别'Jack'以及'open'关键词后向这个节点发布消息"door_open"
*/
ros::Publisher door_pub;
ros::Subscriber speech_sub;
ros::Subscriber pcl_sub;
ros::Subscriber rgb_sub;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frame (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

std::string PCD_DIR = "/home/kamerider/catkin_ws/src/image_pcl/pcd_files";\
std::string PCL_TOPIC_NAME = "/camera/depth/points";
std::string RGB_TOPIC_NAME = "/image_raw";
bool door_closed = false;
bool door_opened = false;
int step = 0;

void pclCallback(const sensor_msgs::PointCloud2& msg)
{
    pcl::fromROSMsg(msg, *cloud_frame);

    //首先剔除点云中Nan点
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud_frame, *cloud_frame, mapping);

    //对点云进行体素滤波下采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud (*cloud_frame, *cloud_temp);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (cloud_temp);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    int num = cloud_filtered->points.size();

    if (num <= 90000)
    {
        door_closed = true;
        step ++;
    }
    else 
    {
        door_opened = true;
        step ++;
    }

    std::cout << "Current Point Number is: ";
    std::cout << num << std::endl;
}

void speechCallback (const std_msgs::StringConstPtr& msg)
{
    /*
    @TODO
    此处根据需要添加对来自语音节点的信息的处理
    */
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr -> image;

    //检测到门是关闭的保存一次数据
    if (door_closed && step==1)
    {
        //std::cout << "Now the door is closed" << endl;
        ROS_INFO ("Door is Closed");
        ROS_INFO ("Writing RGB image");
        std::string FILE_NAME = "/door_closed";
        std::string RGB_FULL_PATH = PCD_DIR + FILE_NAME + ".png";
        cv::imwrite(RGB_FULL_PATH, img);

        //保存PCL数据
        //将当前点云赋予RGB值之后再保存为PCD文件
        pcl::PointXYZRGB temp_point;

        ROS_INFO("Writing PCD files");
        for (int m = 0; m<img.rows; m++)
        {
            for (int n = 0; n<img.cols; n++)
            {
                temp_point.b = img.ptr<uchar>(m,n)[0];
                temp_point.g = img.ptr<uchar>(m,n)[1];
                temp_point.r = img.ptr<uchar>(m,n)[2];

                //取得深度数据
                temp_point.x = cloud_frame->points[m*img.cols+n].x;
                temp_point.y = cloud_frame->points[m*img.cols+n].y;
                temp_point.z = cloud_frame->points[m*img.cols+n].z;
                cloud_rgb->push_back(temp_point);
            }
        }
        std::string PCD_FULL_PATH = PCD_DIR + FILE_NAME + ".pcd";
        pcl::io::savePCDFileASCII(PCD_FULL_PATH, *cloud_rgb);

        //向语音节点发送"door_closed"信息
        std_msgs::String door_flag;
        door_flag.data = "door_closed";
        door_pub.publish(door_flag);

        door_closed = false;
    }
    else if (door_opened && step==2)
    {
        sleep(3);
        //std::cout << "Now the door is closed" << endl;
        ROS_INFO ("Writing RGB image");
        std::string FILE_NAME = "/door_opened";
        std::string RGB_FULL_PATH = PCD_DIR + FILE_NAME + ".png";
        cv::imwrite(RGB_FULL_PATH, img);

        //保存PCL数据
        //将当前点云赋予RGB值之后再保存为PCD文件
        pcl::PointXYZRGB temp_point;

        ROS_INFO("Writing PCD files");
        for (int m = 0; m<img.rows; m++)
        {
            for (int n = 0; n<img.cols; n++)
            {
                temp_point.b = img.ptr<uchar>(m,n)[0];
                temp_point.g = img.ptr<uchar>(m,n)[1];
                temp_point.r = img.ptr<uchar>(m,n)[2];

                //取得深度数据
                temp_point.x = cloud_frame->points[m*img.cols+n].x;
                temp_point.y = cloud_frame->points[m*img.cols+n].y;
                temp_point.z = cloud_frame->points[m*img.cols+n].z;
                cloud_rgb->push_back(temp_point);
            }
        }
        std::string PCD_FULL_PATH = PCD_DIR + FILE_NAME + ".pcd";
        pcl::io::savePCDFileASCII(PCD_FULL_PATH, *cloud_rgb);
        door_opened = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "door_detect");
    ros::NodeHandle nh;

    door_pub   = nh.advertise<std_msgs::String>("/pcl2nav", 1);
    speech_sub = nh.subscribe("/speech2pcl", 1, speechCallback);
    pcl_sub = nh.subscribe("/camera/depth/points", 1, pclCallback);
    rgb_sub = nh.subscribe("/image_raw", 1, imageCallback);

    ros::spin();
    return 0;
}

