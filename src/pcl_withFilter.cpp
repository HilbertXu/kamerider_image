#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//filter file
//体素网格采样
#include <pcl_ros/filters/voxel_grid.h>


/*
PCL点云格式与ROS点云格式的转换
对点云进行体素网格采样
*/
ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    //声明储存原始数据与滤波后点云的容器
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; //原始数据
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered; //滤波后的点云数据

    //由ROS中的点云数据格式转化成为PCL的原始点云数据格式
    //调用滤波函数进行滤波
    pcl_conversions::toPCL(*msg, *cloud);

    //滤波处理
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor; //声明一个滤波实例
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.1, 0.1, 0.1);//设置体素网格大小
    sor.filter(cloud_filtered);

    //再将滤波后的点云由PCL格式装换成为ROS格式
    //声明一个承载转换后ROS格式点云的实例
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    pub.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_withFilter");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("output",1);

    ros::spin();
    return 0;
}