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
using namespace std;


std::string PCD_DIR = "/home/kamerider/catkin_ws/src/image_pcl/pcd_files/";
std::string FILE_NAME;

int main(int argc, char** argv)
{
    FILE_NAME = argv[1];
    std::string RGB_FULL_PATH = PCD_DIR + FILE_NAME + ".png";
    std::string PCL_FULL_PATH = PCD_DIR + FILE_NAME + ".pcd";

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr origin_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr merge_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    //read RGB image
    cv::Mat rgb_image = cv::imread(RGB_FULL_PATH,1);
    //read PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (PCL_FULL_PATH, *origin_cloud_ptr) == -1)
        {
            PCL_ERROR ("Couldn't read PCD file \n");
            return 0;
        }

    //将PCD文件中的位置信息和RGB图像中的RGB信息整合在一起
    pcl::PointXYZRGBA temp_point;
    for (int m=0; m<rgb_image.rows; m++)
    {
        for (int n=0; n<rgb_image.cols; n++)
        {
            //get RGB value
            temp_point.b = rgb_image.ptr<uchar>(m,n)[0];
            temp_point.g = rgb_image.ptr<uchar>(m,n)[1];
            temp_point.r = rgb_image.ptr<uchar>(m,n)[2];

            //get XYZ value
            temp_point.x = origin_cloud_ptr->points[m*rgb_image.cols+n].x;
            temp_point.y = origin_cloud_ptr->points[m*rgb_image.cols+n].y; 
            temp_point.z = origin_cloud_ptr->points[m*rgb_image.cols+n].z;  
            merge_cloud_ptr->push_back(temp_point);
        }
    }

    //写入PCD文件
    pcl::io::savePCDFileASCII(PCL_FULL_PATH, *merge_cloud_ptr);
    std::cout << "PointCloude data has been writen to " << PCL_FULL_PATH << std::endl;

    return 0;

}