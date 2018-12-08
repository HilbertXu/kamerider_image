/*
 * Date: 2018/12/8
 * Author: Xu Yucheng
 * 使用超体聚类算法实现点云分割之前的预处理
 * */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <fstream>
#include <string>
#include <boost/thread/thread.hpp>
#include <cstdlib>
#include <vector>

//Point Cloud Library
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/impl/supervoxel_clustering.hpp>
#include <pcl/segmentation/impl/lccp_segmentation.hpp>

//VTK include to draw graph lines
#include <vtkPolyLine.h>

#typedef pcl::PointXYZRGBA PointT;
#typedef pcl::PointCloud<PointT> PointCloudT;
#typedef pcl::PointNormal PointNT;
#typedef pcl::PointCloud<PointNT> PointCloudNT;
#typedef pcl::PointXYZL PointLT;
#typedef pcl::PointCloud<PointLT> PoinrCloudLT;

std::string PCD_DIR    = "/home/kamerider/catkin_ws/src/image_pcl/pcd_files/";
std::string SEG_OUTPUT = "/home/kamerider/catkin_ws/src/image_pcl/segmentation_output/"; 


using namespace std;
using namespace pcl;

//点云容器
PointCloudT cloud_frame;//暂时储存从ROS中读取的点云数据
PointCloudT::Ptr origin_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);

//ROS系统下的订阅器和发布器
ros::Publisher  sp_pub;
ros::Subscriber pcl_sub;
ros::Subscriber nav_sub;

//SuperVoxel parameters
float voxel_resolution   = 0.008f;
float seed_resolution    = 0.1f;
float color_importance   = 0.2f;
float spatial_importance = 0.4f;
float normal_importance  = 1.0f;



void SuperVoxelSegmentation(PointCloudT::Ptr cloud)
{
    pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform (false);
    super.setInputCloud (cloud);
    super.setColorImportance (color_importance);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (normal_importance);

    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

    std::cout << "-------------------Extracting Supervoxel----------------" << std::endl;
    super.extract (supervoxel_clusters);
    std::cout << "Found " << supervoxel_clusters.size() << " supervoxels" << std::endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer 
                     (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (255, 251, 240);

    //获取体素中心的点云
    PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
    viewer->addPointCloud (voxel_centroid_cloud, "voxel centroid");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "voxel centroid");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "voxel centroid");

    PointCloudLT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
    viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "labeled voxels");
    
    PointCloudNT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);

    //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
    //viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");
    
    std::cout << "Getting Supervoxel Adjacency" << std::endl;

    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency (supervoxel_adjacency);
    
    //为了使我们生成的每个晶元连成一副图，我们需要遍历每一个晶元
    std::multimap<uint32_t, uint32_t>::iterator label_itr = supervoxel_adjacency.begin();

    for( ; label_itr != supervoxel_adjacency.end(); )
    {
        //首先获取迭代器对应的标签
        uint32_t supervoxel_label = label_itr->first;
        //然后找到标签对应的超体
        pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

        //遍历每个超体的邻近，然后生成点云
        PointCloudT adjacency_supervoxel_centers;
        std::multimap<uint_32 uint_32>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
        
        //开始遍历
        for( ; adjacent_itr != supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
        {
            pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
            adjacency_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
        }

        
    }



}

//ROS端回调函数的声明
void pclCallback(const sensor_msgs::PointCloud2& msg)
{
    if(IF_SAVE_PCL)
    {
        //对容器中点云进行复制
        //使用从ROS类型到PCL类型的转换函数
        pcl::fromROSMsg(msg, cloud_frame);
        *origin_cloud_ptr = cloud_frame;
        IF_SAVE_PCL = false;
    }


}

void navCallback(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == "arrived_at_observe_position")
    {
        //到达指定导航点之后开始拍摄照片，并记录点云数据
        IF_SAVE_PCL = true;
    }
}

void printUsage(const char* program_name)
{
    std::cout << "\n\nUsage: rosrun robot_vision pcl_keypoint_detect <pcd file name> -[options]\n\n"
                << "Options:\n"
                << "-------------------------------------------\n"
                << "-help           this help\n"
                << "-s              SIFT keypoint detection example\n"
                << "-h              Harris keypoint detection example\n"
                << "-n              NARF keypoint detection example\n"
                << "-i              ISS keypoint detection example\n"
                << "\n\n";
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        printUsage(argv[0]);
    }
    
    if (command == "-h")
	{
		//argv[0]储存了当前运行程序的名称
		printUsage(argv[0]);
		return 0;
	}

    if (command == "-ros")
    {
        /*
        @TODO
        integrate with ROS
        */
        std::cout << "---------------Using ROS Mode Initial ROS Node---------------" << std::endl;
        std::cout << "--------------------Waiting for message----------------------" << std::endl;
        ros::init(argc, argv, "pcl_supervoxel_lccp");
        ros::NodeHandle nh;

        sp_pub  = nh.advertise<std_msgs::String>("/pcl2speech", 1);
        pcl_sub = nh.subscribe("/camera/depth/points", 1, pclCallback);
        nav_sub = nh.subscribe("/nav2pcl", 1, navCallback);

        ros::spin();
    }

    else if(command == "-pcd")
    {
        std::cout << "-------------------Read PCL From PCD Files-------------------" << std::endl;
        std::cout << "---------------Reading " << argv[1] << " file-------------" << std::endl;
        std::string FILE_NAME = argv[1];
        std::string FULL_PATH = PCD_DIR + FILE_NAME;

        if (pcl::io::loadPCDFile<PointXYZRGBA> (FULL_PATH, *origin_cloud_ptr) == -1)
        {
            PCL_ERROR ("Couldn't read PCD file \n");
            return 0;
        }

        //在读取了PCD文件之后先剔除Nans
        //避免后续算法调用的时候出现访问错误
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, mapping);

    }
        

    return 0;
}

