#include <iostream> 
#include <string> 
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/range_image/range_image.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

//体素网格采样
#include <pcl_ros/filters/voxel_grid.h>
using namespace std; 
using namespace pcl;



int main (int argc, char** argv)
{ 
    typedef pcl::PointXYZRGBA PointT; 
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>); 

    std::cout << argv[1];

    std::string dir = "/home/nvidia/catkin_ws/src/kamerider_image/kamerider_image_pcl/pcd_files/"; 
    std::string filename = argv[1]; 

    if (pcl::io::loadPCDFile<PointT> ((dir+filename), *cloud) == -1)
    { 
        //* load the file 
        PCL_ERROR ("Couldn't read PCD file \n"); 
        return (-1); 
    } 
    printf("Loaded %d data points from PCD\n", 
    cloud->width * cloud->height); 

    for (size_t i = 0; i < cloud->points.size (); i+=10000) 
    printf("%8.3f %8.3f %8.3f %5d %5d %5d %5d\n", 
    cloud->points[i].x, 
    cloud->points[i].y, 
    cloud->points[i].z, 
    cloud->points[i].r, 
    cloud->points[i].g, 
    cloud->points[i].b, 
    cloud->points[i].a 
    ); 

    //增加剔除Nans点以及点云滤波部分
    //在读取了PCD文件之后先剔除Nans
    //避免后续算法调用的时候出现访问错误
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);

    /*
    取消注释以实现对点云进行体素滤波下采样
    
    std::cout << "Using VoxelGrid Filter" << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud (*origin_cloud_ptr, *temp_cloud);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (temp_cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter(*filtered_cloud_ptr); 
    */

    pcl::visualization::PCLVisualizer viewer("Cloud viewer"); 
    viewer.setBackgroundColor(255,251,240);
    //设置一个自定义的颜色处理器
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(*cloud, 0, 255, 0);
    viewer.setCameraPosition(0,0,-3.0,0,-1,0); 
    viewer.addCoordinateSystem(0.3); 

    //viewer.addPointCloud(cloud, single_color); 
    viewer.addPointCloud(cloud);
    while(!viewer.wasStopped()) 
        viewer.spinOnce(100); 
    return (0); 
} 

