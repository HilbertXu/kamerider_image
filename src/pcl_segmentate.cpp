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

#include <pcl/filters/passthrough.h>

//点云分割有关头文件
//分别为超体聚类和LCCP算法
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#define Random(x) (rand() % x)

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

void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: rosrun robot_vision pcd_visualization <pcd file name> -[options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}

void pcl_segmentation_with_LCCP(pcl::PointCloud<PointT>::Ptr cloud)
{
    //算法思想是先用超体聚类方法进行一次过分割
    //超体聚类算法类似于晶体结晶的过程，设置结晶核大小，之后结晶核会通过不同块之间的凹凸关系判断
    //其中使用了两种约束条件 CC（Extended Convexity Criterion） 和 SC （Sanity criterion）
    //然后再使用LCCP(Locally Convex Connected Patches)算法进行聚类
    //将过分割的情况转换成正常分割的情况
    pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform(use_single_cam_transform);
    super.setInputCloud(origin_cloud_ptr);  
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
 
	PCL_INFO("Extracting supervoxels\n");
	super.extract(supervoxel_clusters);
 
	PCL_INFO("Getting supervoxel adjacency\n");
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);

    pcl::PointCloud<pcl::PointXYZL>::Ptr over_seg = super.getLabeledCloud();

    //读写文件操作
    ofstream outFile1((OUTPUT_DIR+"over_seg.txt").c_str(), std::ios_base::out);
    for (int i=0; i<over_seg->size(); i++)
    {        
        outFile1 << over_seg->points[i].x << "\t" << over_seg->points[i].y << "\t" 
                 << over_seg->points[i].z << "\t" << over_seg->points[i].label << std::endl;
    }

    int label_max1 = 0;
    for (int i=0; i<over_seg->size(); i++)
    {
        if (over_seg->points[i].label > label_max1)
        {
            label_max1 = over_seg->points[i].label;
        }
    }

    //对点云进行染色，区分第一次过分割分割的结果
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    ColoredCloud1->height = 1;
    ColoredCloud1->width = over_seg->size();
    ColoredCloud1->resize(over_seg->size());
    for (int i=0; i < label_max1; i++)
    {
        int color_R = Random(255);
        int color_G = Random(255);
        int color_B = Random(255);

        for(int j=0; j < over_seg->size(); j++)
        {
            if(over_seg->points[j].label == i)
            {
                ColoredCloud1->points[j].x = over_seg->points[j].x;
                ColoredCloud1->points[j].y = over_seg->points[j].y;
                ColoredCloud1->points[j].z = over_seg->points[j].z;
                //RGB
                ColoredCloud1->points[j].r = color_R;
                ColoredCloud1->points[j].g = color_G;
                ColoredCloud1->points[j].b = color_B;
            }
        }
    }
    
	pcl::io::savePCDFileASCII(PCD_DIR + "over_seg.pcd", *ColoredCloud1);

    //LCCP分割
    PCL_INFO ("Start Segmentation\n");
    pcl::LCCPSegmentation<PointT> lccp;
    lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	lccp.setMinSegmentSize(min_segment_size);
    lccp.segment();

    PCL_INFO ("Interpolation voxel cloud -> Input cloud and relabeling\n");

    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud   = super.getLabeledCloud();
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
    lccp.relabelCloud(*lccp_labeled_cloud);
    SupervoxelAdjacencyList sv_adjacency_list;
    lccp.getSVAdjacencyList(sv_adjacency_list);

    ofstream outFile2 ((OUTPUT_DIR + "overSeg_merge.txt").c_str(), std::ios_base::out);
    for (int i=0; i < lccp_labeled_cloud->size(); i++)
    {
        outFile2 << lccp_labeled_cloud->points[i].x << "\t" << lccp_labeled_cloud->points[i].y << "\t"
                 << lccp_labeled_cloud->points[i].z << "\t" << lccp_labeled_cloud->points[i].label << std::endl;
    }

    int label_max2 = 0;
    for (int i=0; i < lccp_labeled_cloud->size(); i++)
    {
        if(lccp_labeled_cloud->points[i].label > label_max2)
        {
            label_max2 = lccp_labeled_cloud->points[i].label;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
    ColoredCloud2->height = 1;
    ColoredCloud2->width  = lccp_labeled_cloud->size();
    ColoredCloud2->resize(lccp_labeled_cloud->size());

    for (int i = 0; i < label_max2; i++) 
    {
		int color_R = Random(255);
		int color_G = Random(255);
		int color_B = Random(255);

		for (int j = 0; j < lccp_labeled_cloud->size(); j++) 
        {
			if (lccp_labeled_cloud->points[j].label == i) 
            {
				ColoredCloud2->points[j].x = lccp_labeled_cloud->points[j].x;
				ColoredCloud2->points[j].y = lccp_labeled_cloud->points[j].y;
				ColoredCloud2->points[j].z = lccp_labeled_cloud->points[j].z;
				ColoredCloud2->points[j].r = color_R;
				ColoredCloud2->points[j].g = color_G;
				ColoredCloud2->points[j].b = color_B;
			}
		}
    }
    pcl::io::savePCDFileASCII (PCD_DIR + "overSeg_merge.pcd", *ColoredCloud2);

    //分割结果可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(255,251,240);
    viewer->setCameraPosition(0,0,-3.0,0,-1,0);
    viewer->addCoordinateSystem(0.3);
    viewer->addPointCloud(ColoredCloud2, "lccp");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "lccp");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

int main(int argc, char **argv)
{
    if (pcl::console::find_argument (argc, argv, "-h")>=0)
	{
		//argv[0]储存了当前运行程序的名称
		printUsage(argv[0]);
		return 0;
	}
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
        std::string FULL_PATH = PCD_DIR + FILE_NAME;

        if (pcl::io::loadPCDFile<PointXYZRGBA> (FULL_PATH, *origin_cloud_ptr) == -1)
        {
            PCL_ERROR ("Couldn't read PCD file \n");
            return 0;
        }
    }


    std::cout << "test lccp header" << endl;
    return 0;
}
