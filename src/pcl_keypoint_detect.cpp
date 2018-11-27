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
#include <pcl\io\pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <cstdlib>
#include <vector>

using namespace pcl;
using namespace std;

void printUsage(const char* program_name)
{
    std::cout << "\n\nUsage: rosrun robot_vision pcl_keypoint_detect <pcd file name> -[options]\n\n"
                << "Options:\n"
                << "-------------------------------------------\n"
                << "-help           this help\n"
                << "-s           SIFT keypoint detection example\n"
                << "-h           Harris keypoint detection example\n"
                << "-n           NARF keypoint detection example\n"
                << "-i           ISS keypoint detection example\n"
                << "\n\n";
}

int main(int argc, char **argv)
{
    if (pcl::console::find_argument (argc, argv, "-help") >=0)
    {
        printUsage(argv[0]);
        return 0;
    }

    bool sift(false), harris(false), narf(false), iss(false);

    if (pcl::console::find_argument (argc, argv, "-s")
    {
        sift = true;
        std::cout << "SIFT keypoint detection example" << std::endl;
    }

    else if (pcl::console::find_argument (argc, argv, "-h"))
    {
        harris = true;
        std::cout << "Harris keypoint detection example" << std::endl;
    }

    else if (pcl::console::find_argument (argc, argv, "-n"))
    {
        narf = true;
        std::cout << "NARF keypoint detection example" << std::endl;
    }

    else if (pcl::console::find_argument (argc, argv, "-i"))
    {
        iss = true;
        std::cout << "ISS keypoint detection example" << std::endl;
    }

    else 
    {
        printUsage (argv[0]);
        return 0;
    }

    //-----------------------------------------------
    //--------read point cloud from PCD file---------
    //-----------------------------------------------

    pcl::PointCloud<PointXYZ>::Ptr origin_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);


    //记录读取的PCD文件的路径
	std::string dir = "/home/kamerider/catkin_ws/src/robot_vision/pcd_files/"; 
  	std::string filename = argv[1];
    std::cout << "Loading " << argv[1];

    if (pcl::io::loadPCDFile<PointXYZ> ((dir + filename), *origin_cloud_ptr) == -1)
    {
        PCL_ERROR ("Couldn't read PCD file \n");
        return 0;
    }


}