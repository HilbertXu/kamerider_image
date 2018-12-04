#include <iostream> 
#include <string> 
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 
#include <pcl/visualization/pcl_visualizer.h> 
using namespace std; 
using namespace pcl;



int main (int argc, char** argv)
{ 
    typedef pcl::PointXYZRGBA PointT; 
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>); 

    std::cout << argv[1];

    std::string dir = "~/catkin_ws/src/robot_vision/pcd_files/"; 
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

