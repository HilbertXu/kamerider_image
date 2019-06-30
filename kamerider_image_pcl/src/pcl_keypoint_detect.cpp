/*
对于多种点云关键点检测方法的时下
包括:iss&Trajkovic Harris NARF SIFT
Author： Xu Yucheng
Github: https://github.com/HilbertXu/PCL_test.git
*/

/*
点云相关头文件/usr/include/pcl-1.7/pcl
*/
#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>
#include <pcl/io/io.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/range_image/range_image.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <cstdlib>
#include <vector>

using namespace pcl;
using namespace std;

//参数 全局变量
//NARF parameters
float angular_resolution = 0.5f; //角坐标分辨率
float support_size = 0.2f; //感兴趣点的尺寸（球面的直径）
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; //坐标框架：相机框架（而不是激光框架）
bool setUnseenToMaxRange = false; //是否将所有不可见的点 看作 最大距离


void printUsage(const char* program_name)
{
    std::cout << "\n\nUsage: rosrun image_pcl pcl_keypoint_detect -[options] <pcd file name> \n\n"
                << "Options:\n"
                << "-------------------------------------------\n"
                << "-help           this help\n"
                << "-s              SIFT keypoint detection example\n"
                << "-h              Harris keypoint detection example\n"
                << "-n              NARF keypoint detection example\n"
                << "-i              ISS keypoint detection example\n"
                << "\n\n";
}

void HarrisKeypoint(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{

    // 3D点云显示
    //初始化可视化对象的指针
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(255,251,240);
    viewer->setCameraPosition(0,0,-3.0,0,-1,0); 
    viewer->addCoordinateSystem(0.3); 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> origin_color_handler (cloud, 0,0,0);
    viewer->addPointCloud(cloud, origin_color_handler);

    // --------------------------------
    // -----Extract Harri keypoints-----
    // --------------------------------
    // 提取Harri关键点
    std::cout << "Receving Point Cloud with size of (" << cloud->height << "," << cloud->width << ")"<<std::endl;
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
    harris.setInputCloud(cloud);//设置输入点云的指针
    std::cout << "Successfully load Point Cloud for harris" << std::endl;
    harris.setNonMaxSupression(true);
    harris.setRadius(0.02f);//设置每次检测的半径
    harris.setThreshold(0.02f);//设置数量阈值
    std::cout << "Successfully set Parameters for harris " << std::endl;


    //新建的点云必须初始化，清零，否则指针访问会越界
    //注意Harris的输出点云必须是有强度(I)信息的，因为评估值保存在I分量里，即需要使用pcl::PointXYZI的数据格式
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>& cloud_out = *cloud_out_ptr;

    cloud_out.height = cloud->height;
    cloud_out.width  = cloud->width;
    cloud_out.resize(cloud_out.height * cloud_out.width);
    cloud_out.clear();
    std::cout << "Extravting......" << endl;

    harris.compute(cloud_out);

    //可视化结果不支持PointXYZI格式的点云，所以又要转换回PointXYZ格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_harris_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& cloud_harris = *cloud_harris_ptr;
    cloud_harris.height = cloud->height;
    cloud_harris.width  = cloud->width;
    cloud_harris.resize(cloud_harris.height * cloud_harris.width);
    cloud_harris.clear();
    //提取Harris特征点
    int size = cloud_out.size();

    std::cout << "Extracted " << size << " n keypoints" << std::endl;
    pcl::PointXYZ point;
    for (int i=0; i<size; i++)
    {
        point.x = cloud_out.at(i).x;
        point.y = cloud_out.at(i).y;
        point.z = cloud_out.at(i).z;
        cloud_harris.push_back(point);
    }

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    //在3D图形窗口中显示关键点
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler (cloud_harris_ptr, 0,255,0);
    viewer->addPointCloud(cloud_harris_ptr, harris_color_handler, "harris");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "harris");

    while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}


//void NarfKeypoint(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{

    /*
    @TODO
    Narf keypoint detect
    似乎需要两种点云文件来实现
    一种视普通的，一种的far_range的点云
    有待后续查阅文档
    */

    // 3D点云显示
    //初始化可视化对象的指针
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
//    viewer->setBackgroundColor(255,251,240);
//    viewer->setCameraPosition(0,0,-3.0,0,-1,0); 
//    viewer->addCoordinateSystem(0.3); 
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> origin_color_handler (cloud, 0,0,0);
//    viewer->addPointCloud(cloud, origin_color_handler);

//    // -----------------------------------------------
//    // -----Create RangeImage from the PointCloud-----
//    // -----------------------------------------------
//    float noise_level = 0.0;
//    float min_range = 0.0f;
//    int border_size = 1;
//    boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
//    pcl::RangeImage& range_image = *range_image_ptr;
//    range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
//                                  scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);   
//}


void SiftKeypoint(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // Parameters for sift computation
    const float min_scale = 0.005f; //the standard deviation of the smallest scale in the scale space
    const int n_octaves = 6;//the number of octaves (i.e. doublings of scale) to compute
    const int n_scales_per_octave = 4;//the number of scales to compute within each octave
    const float min_contrast = 0.005f;//the minimum contrast required for detection
    
    // 3D点云显示
    //初始化可视化对象的指针
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(255,251,240);
    viewer->setCameraPosition(0,0,-3.0,0,-1,0); 
    viewer->addCoordinateSystem(0.3); 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> origin_color_handler (cloud, 0,0,0);
    viewer->addPointCloud(cloud, origin_color_handler);

    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    //设置搜索模式为KdTree
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud);
    sift.compute(result);

    std::cout << "Extracted " << result.size() << " n keypoints" << std::endl;

    //为了可视化需要将PointWithScale格式点云转化为PointXYZ格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sift_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& cloud_sift = *cloud_sift_ptr; 
    
    //可以尝试使用点云复制函数
    //copyPointCloud(result, *cloud_sift);
    pcl::PointXYZ point;
    for(int i=0; i<result.size();i++)
    {
        point.x = result.at(i).x;
        point.y = result.at(i).y;
        point.z = result.at(i).z;
        cloud_sift.push_back(point);
    }

    //可视化
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sift_color_handler (cloud_sift_ptr, 0,255,0);
    viewer->addPointCloud(cloud_sift_ptr, sift_color_handler, "sift");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sift");

    while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
}


int main(int argc, char **argv)
{
    for(int i=0; i<argc; i++)
    {
        cout << "argv[" << i << "]: " << argv[i] << "\n";
        // printf("%p",argv[2]);
    }

    std::cout << endl;
    if (argv[2] ==  "-help")
    {
        printUsage(argv[0]);
        return 0;
    }
    /*
     * 智能的cout会直接输出这个指针指向的内存区域中储存的数据
     * argv[2]返回的是一个指针，直接调用的话argv[i]的值会是一个地址值
     * 所以需要用一个string类型将argv[i]接住之后再进行比较
     * */
    std::string command = argv[1];

    bool sift(false), harris(false), narf(false), iss(false);
    if (command ==  "-s")
    {
        sift = true;
        std::cout << "SIFT keypoint detection example" << std::endl;
    }

    else if (command ==  "-h")
    {
        harris = true;
        std::cout << "Harris keypoint detection example" << std::endl;
    }

    else if (command ==  "-n")
    {
        narf = true;
        std::cout << "NARF keypoint detection example" << std::endl;
    }

    else if (command ==  "-i")
    {
        iss = true;
        std::cout << "ISS keypoint detection example" << std::endl;
    }

    else 
    {
        std::cout << "No command received " << std::endl;
        printUsage (argv[0]);
        return 0;
    }

    //-----------------------------------------------
    //--------read point cloud from PCD file---------
    //-----------------------------------------------

    pcl::PointCloud<PointXYZ>::Ptr origin_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    //记录读取的PCD文件的路径
	std::string dir = "/home/nvidia/catkin_ws/src/image_pcl/pcd_files/";
    std::string filename = argv[2];
    std::cout << "Loading " << argv[2];
   

    if (pcl::io::loadPCDFile<PointXYZ> ((dir + filename), *origin_cloud_ptr) == -1)
    {
        PCL_ERROR ("Couldn't read PCD file \n");
        return 0;
    }

    
    if(sift)
    {
        SiftKeypoint(origin_cloud_ptr);
    }

    else if(harris)
    {
        HarrisKeypoint(origin_cloud_ptr);
    }

    return 0;
}
