#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
//体素网格采样
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
 
int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_leaf (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;

    std::string PCD_DIR = "/home/nvidia/catkin_ws/src/image_pcl/pcd_files/";
    std::string FULL_PATH = PCD_DIR + argv[1];
    // 读入点云PCD文件
    reader.read(FULL_PATH,*cloud);

    //对点云进行体素滤波下采样
    //减少计算量
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud (*cloud, *cloud_temp);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (cloud_temp);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_leaf);
    pcl::copyPointCloud(*cloud_leaf, *cloud);
    
    //首先使用ransac拟合地面
    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // 距离阈值 单位m
    seg.setDistanceThreshold (0.001f);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
 
 
    // 提取地面
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter (*cloud_filtered);
 
    std::cerr << "Ground cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
 
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ((PCD_DIR + "ransac_seg_ground.pcd"), *cloud_filtered, false);
 
    // 提取除地面外的物体
    extract.setNegative (true);
    extract.filter (*cloud_filtered);
 
    std::cerr << "Object cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
 
    writer.write<pcl::PointXYZ> ((PCD_DIR + "ransac_seg_object.pcd"), *cloud_filtered, false);
 
    // 点云可视化
    pcl::visualization::PCLVisualizer viewer("Filtered");
    viewer.setBackgroundColor(255,251,240);
    //设置一个自定义的颜色处理器
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(*cloud, 0, 255, 0);
    viewer.setCameraPosition(0,0,-3.0,0,-1,0); 
    viewer.addCoordinateSystem(0.3); 

    //viewer.addPointCloud(cloud, single_color); 
    viewer.addPointCloud(cloud_filtered);
    while(!viewer.wasStopped()) 
        viewer.spinOnce(100); 
    return (0);
}
