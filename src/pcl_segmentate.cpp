/*
基于LCPP(Locally Convex Connected Patches)和超体聚类算法的点云分割
Author： Xu Yucheng
Github: https://github.com/HilbertXu/PCL_test.git
头文件路径：/usr/include/pcl-1.7/pcl/segmentation
*/
#include <iostream>
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

int main()
{
    std::cout << "test lccp header" << endl;
    return 0;
}