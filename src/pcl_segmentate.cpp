/*
基于LCPP(Locally Convex Connected Patches)和超体聚类算法的点云分割
Author： Xu Yucheng
Github: https://github.com/HilbertXu/PCL_test.git
头文件路径：/usr/include/pcl-1.7/pcl/segmentation
*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <fstream>
#include <string>
#include <boost/thread/thread.hpp>
#include <cstdlib>
#include <vector>

#include <pcl/io/io.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/range_image/range_image.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h> 

//点云分割有关头文件
//分别为超体聚类和LCCP算法
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SupervoxelAdjacencyList;

//全局变量
bool IF_SAVE_PCL = false;
std::string PCD_DIR = "/home/kamerider/catkin_ws/src/robot_vision/pcd_files/";
std::string OUTPUT_DIR = "/home/kamerider/catkin_ws/src/robot_vision/segmentation_output/";
pcl::PointCloud<PointT> cloud_frame;//暂时储存从ROS中读取的点云数据
pcl::PointCloud<PointT>::Ptr origin_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);

//使用Supervoxel时的参数
float voxel_resolution = 0.3f;
float seed_resolution = 1.2f;
float color_importance = 0.0f;
float spatial_importance = 1.0f;
float normal_importance = 0.0f;
bool use_single_cam_transform = false;
bool use_supervoxel_refinement = false;
unsigned int k_factor = 0;

//使用LCCP对Supervoxel分割后的点云进行处理时的参数
float concavity_tolerance_threshold = 10;
float smoothness_threshold = 0.1;
uint32_t min_segment_size = 0;
bool use_extended_convexity = false;
bool use_sanity_criterion = false;

//ROS系统下的订阅器和发布器
ros::Publisher sp_pub;
ros::Subscriber pcl_sub;
ros::Subscriber nav_sub;
//ROS端回调函数的声明
void pclCallback(const sensor_msgs::PointCloud2& msg)
{
    if(IF_SAVE_PCL)
    {
        //对容器中点云进行复制
        //使用从ROS类型到PCL类型的转换函数
        pcl::fromROSMsg(msg, cloud_frame);
        origin_cloud_ptr = &cloud_frame;
        IF_SAVE_PCL = false;
    }

    
}
void navCallback(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == "arrived_at_observe_position")
    {
        IF_SAVE_PCL = true;
    }
}

void pcl_segmentation_with_LCCP(pcl::PointCloud<PointT>::Ptr cloud)
{
    //算法思想是先用超体聚类方法进行一次过分割
    //超体聚类算法类似于晶体结晶的过程，设置结晶核大小，之后结晶核会通过不同块之间的凹凸关系判断
    //其中使用了两种约束条件 CC（Extended Convexity Criterion） 和 SC （Sanity criterion）
    //然后再使用LCCP(Locally Convex Connected Patches)算法进行聚类
    //将过分割的情况转换成正常分割的情况
}

int main(int argc, int argv)
{
    if(argc < 2)
    {
        /*
        @TODO
        integrate with ROS
        */
        std::cout << "---------------Using ROS Mode Initial ROS Node---------------" << std::endl;
        std::cout << "--------------------Waiting for message----------------------" << std::endl;
        ros::init(argc, argv, "pcl_segmentate");
        ros::NodeHandle nh;

        sp_pub  = nh.advertise<std_msgs::String>("/pcl2speech", 1);
        pcl_sub = nh.subscribe("/camera/depth/points", 1, pclCallback);
        nav_sub = nh.subscribe("/nav2pcl", 1, navCallback);

        ros::spin();
    }
    else
    {
        std::cout << "-------------------Read PCL From PCD Files-------------------" << std::endl;
        std::cout << "---------------Reading " << argv[1] << " file-------------" << std::endl;
        std::string FILE_NAME = argv[1];
        std::full_path = PCD_DIR + FILE_NAME;

        if (pcl::io::loadPCDFile<PointXYZRGBA> ((dir + filename), *origin_cloud_ptr) == -1)
        {
            PCL_ERROR ("Couldn't read PCD file \n");
            return 0;
        }
    }


    std::cout << "test lccp header" << endl;
    return 0;
}