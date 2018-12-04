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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//filter file
//体素网格采样
#include <pcl_ros/filters/voxel_grid.h>

using namespace std;
using namespace cv;

//定义全局变量来储存接受到的点云
pcl::PointCloud<pcl::PointXYZRGB> cloud_frame;
string PCD_PATH = "~/catkin_ws/src/robot_vision/pcd_files";

//保存image和pcd时的计数器
int pCount = 0;


//定义订阅器和发布器
ros::Subscriber pcl_sub;
ros::Subscriber img_sub;

string int2str(int &int_temp)
{
    stringstream stream;
    stream<<int_temp;
    string string_temp = stream.str();

    return string_temp;
}

void writePCD(string path)
{
    pcl::io::savePCDFileASCII(path, cloud_frame);
    std::cout << "PointCloude data has been writen to " << path << std::endl;
}

void pclCallback(const sensor_msgs::PointCloud2& msg)
{
    //对容器中点云进行复制
    //使用从ROS类型到PCL类型的转换函数
    pcl::fromROSMsg(msg, cloud_frame);

}

/*
@TODO
将按下‘s’键保存当前RGB图片和点云数据修改成为
接受到导航节点发送过来的已经到达目标点的消息时保存数据
同时增加向语音节点发送消息的发布器，通过语音节点实现人机交互
*/

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr -> image;
    cv::namedWindow("demo", CV_WINDOW_AUTOSIZE);
    cv::imshow("demo", img);
    int key = cv::waitKey(10);
    if(key == 's')//按下s键将当前点云数据保存到pcd文件中
    {
        pCount++;

        ROS_INFO("Writing PCD files");
        string num = int2str(pCount);
        string pcd_path = PCD_PATH + "/" + num + ".pcd";
        writePCD(pcd_path);

        string img_path = PCD_PATH + "/" + num + ".png";

        ROS_INFO("Writing RGB image");
        cv::imwrite(img_path, img);
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_pcl_as_pcd");
    ros::NodeHandle nh;

    pcl_sub = nh.subscribe("/camera/depth/points", 1, pclCallback);
    img_sub = nh.subscribe("/image_raw", 1, imageCallback);

    ros::spin();
    return 0;
}

