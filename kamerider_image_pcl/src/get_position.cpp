#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <stdlib.h>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;
using namespace cv;
using namespace pcl;

//定义发布器&订阅器 以及全局变量
//订阅语音话题，准备转动机器人
//ros::Subscriber sp_sub;
//ros::Subscriber nav_sub;
ros::Subscriber img_sub;
ros::Subscriber pcl_sub;
pcl::PointCloud<pcl::PointXYZ> Cloud_Frame;
unsigned char floatBuffer[4];
//图像储存完成之后向PUBLISH_RET_TOPIC_NAME发布消息
ros::Publisher pcl_pub;

int image_width = 640;
int image_height = 480;
//ros::Publisher gPublisher;
//ros::Publisher move_pub;
//ros::Publisher taking_photo_Pub;

//当存在点云数据是Nans时，从邻近点云中找到非Nan的点云替代
int FindNearValid(int Idx)
{
	pcl::PointCloud<pcl::PointXYZ>*pclPt=NULL;
	pclPt = &Cloud_Frame;

	int Crow=Idx/image_width;
	int Ccol=Idx%image_width;
	double TmpMin=9e300;
	int returnIdx=Idx;
	for(int row=0;row<image_width;row++)
	{
		for(int col=0;col<image_width;col++)
		{
			if((!isnan(pclPt->points[row*image_width+col].x))&&
				(!isnan(pclPt->points[row*image_width+col].y))&&
				(!isnan(pclPt->points[row*image_width+col].z)))
			{
				double Dis2=(row-Crow)*(row-Crow)+(col-Ccol)*(col-Ccol);
				if(Dis2<TmpMin)
				{
					returnIdx=row*image_width+col;
					TmpMin=Dis2;
				}
			}
		}
	}
	return returnIdx;
}

void pclCallback(sensor_msgs::PointCloud2 msg)
{
    //ROS_INFO("RECEIVING PCL DATA");
    sensor_msgs::PointCloud2 oMsg;
    Cloud_Frame.width  = msg.width;
    Cloud_Frame.height = msg.height;
    Cloud_Frame.points.resize(Cloud_Frame.width * Cloud_Frame.height);
    //cout << "The Size of Cloud_Frame is " <<Cloud_Frame.size();
    for(size_t i = 0; i<Cloud_Frame.size(); i++)
    {
        floatBuffer[0] = msg.data[i*16+0];
        floatBuffer[1] = msg.data[i+16+1];
        floatBuffer[2] = msg.data[i*16+2];
        floatBuffer[3] = msg.data[i*16+3];
        double X = *((float*)floatBuffer);

        floatBuffer[0] = msg.data[i*16+4];
        floatBuffer[1] = msg.data[i+16+5];
        floatBuffer[2] = msg.data[i*16+6];
        floatBuffer[3] = msg.data[i*16+7];
        double Y = *((float*)floatBuffer);

        floatBuffer[0] = msg.data[i*16+8];
        floatBuffer[1] = msg.data[i+16+9];
        floatBuffer[2] = msg.data[i*16+10];
        floatBuffer[3] = msg.data[i*16+11];
        double Z = *((float*)floatBuffer);

        Cloud_Frame.points[i].x = X;
        Cloud_Frame.points[i].y = Y;
        Cloud_Frame.points[i].z = Z;
    }
    pcl::toROSMsg(Cloud_Frame, oMsg);
    oMsg.header.frame_id = "camera_link";
    pcl_pub.publish(oMsg);
}
/*
@TODO
将鼠标点击操作替换为以下两种可能的操作
1. Darknet识别物体标出bounding_boxes之后取中电作为抓取点
2. 尝试使用点云进行点云配准操作，即预先读取要抓取的物体的点云信息，
   然后通过Hough算法在大环境的点云数据中找到这个物体的一部分
*/
//定义回调函数，返回鼠标每次点击时对应点的坐标值以及RGB值
void on_mouse(int event, int x, int y, int flags, void* param)
{
    Mat *im = reinterpret_cast<Mat*>(param);

    switch(event)
    {
        case CV_EVENT_LBUTTONDOWN:
            //左键按下响应：输出并保存点击处的坐标
            int row = y;
            int col = x;
            int idx = row*image_width + col;
            int new_idx = 0;
            //if(isnan(Cloud_Frame.points[idx].x) || isnan(Cloud_Frame.points[idx].y) || isnan(Cloud_Frame.points[idx].z))
            //{
            //    new_idx = FindNearValid(idx);
            //}
            //从原始点云中移除Nans
            //std::vector<int> mapping;
            //removeNaNFromPointCloud(Cloud_Frame, Cloud_Frame, mapping);
            std::cout << "at(" << x << "," << y << ")" <<std::endl;
            std::cout << "3D position(" << Cloud_Frame.points[new_idx].x <<","
                                        << Cloud_Frame.points[new_idx].y << "," 
                                        << Cloud_Frame.points[new_idx].z << ")" <<std::endl;     
            break;

    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr -> image;
    cv::namedWindow("demo", CV_WINDOW_AUTOSIZE);
    cv::imshow("demo", img);
    cv::waitKey(3);

    //调用之前声明的回调函数on_mouse
    cv::setMouseCallback("demo",on_mouse,reinterpret_cast<void*> (&img));

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PCL_test");
    ROS_INFO("--------INITIALZING--------");

    ros::NodeHandle nh;
    std::cout << "subscribe pcl topic" << std::endl;
    pcl_sub = nh.subscribe("/camera/depth/points", 1, pclCallback);
    
    std::cout << "publish pcl message" << std::endl;
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/astra_pcl",1);
    
    std::cout << "subscribe RGB image" << std::endl;
    img_sub = nh.subscribe("/image_raw", 1, imageCallback);

    ros::spin();
    return 0;
}