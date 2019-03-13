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

//filter file
//体素网格采样
#include <pcl_ros/filters/voxel_grid.h>

using namespace std;
using namespace cv;

//定义全局变量来储存接受到的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frame (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
string PCD_PATH = "/home/nvidia/catkin_ws/src/image_pcl/pcd_files";
string PCL_TOPIC_NAME;
string RGB_TOPIC_NAME;

//保存image和pcd时的计数器
int pCount = 0;


//定义订阅器和发布器
ros::Subscriber pcl_sub;
ros::Subscriber img_sub;

void printUsage(const char* name)
{

  std::cout << "\n\nUsage: rosrun image_pcl save_pcl_as_pcd [PCL_TOPIC_NAME] [RGB_TOPIC_NAME] \n\n"
            << "\n\n";
}

string int2str(int &int_temp)
{
    stringstream stream;
    stream<<int_temp;
    string string_temp = stream.str();

    return string_temp;
}

void writePCD(string path)
{
    pcl::io::savePCDFileASCII(path, *cloud_rgb);
    std::cout << "PointCloude data has been writen to " << path << std::endl;

    //添加可视化模块，每次保存之后显示当前保存的点云
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(255,251,240);
    viewer->setCameraPosition(0,0,-3.0,0,-1,0);
    viewer->addCoordinateSystem(0.3);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
    viewer->addPointCloud(cloud_rgb, rgb, "point_cloud");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point_cloud");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void pclCallback(const sensor_msgs::PointCloud2& msg)
{
    //对容器中点云进行复制
    //使用从ROS类型到PCL类型的转换函数
    pcl::fromROSMsg(msg, *cloud_frame);
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
        string num = int2str(pCount);

        string img_path = PCD_PATH + "/" + num + ".png";
        ROS_INFO("Writing RGB image");
        cv::imwrite(img_path, img);
        
        //将当前点云赋予RGB值之后再保存为PCD文件
        pcl::PointXYZRGB temp_point;

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
        ROS_INFO("Writing PCD files");
        string pcd_path = PCD_PATH + "/" + num + ".pcd";
        writePCD(pcd_path);
    }

}

int main(int argc, char **argv)
{
    //if (argc < 3)
    //{
    //    printUsage(argv[0]);
    //    return 0;
    //}

    //PCL_TOPIC_NAME = argv[1];
    //RGB_TOPIC_NAME = argv[2];
    ros::init(argc, argv, "save_pcl_as_pcd");
    ros::NodeHandle nh;

    pcl_sub = nh.subscribe("/camera/depth/points", 1, pclCallback);
    img_sub = nh.subscribe("/image_raw", 1, imageCallback);

    ros::spin();
    return 0;
}

