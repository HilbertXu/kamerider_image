/*
Used for GPSR(General Purpose Service Robot)
Date: 2018/12/14
Author: Xu Yucheng
在到达导航地点之后开始寻找物体，并进行机器人位置的微调
目标是将机器人调整到正对识别出来的物体
但是需要考虑同时识别出多个物体的情况
*/
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>
#include <cmath>

#include <boost/thread/thread.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
//体素网格采样
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

//ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <darknet_ros_msgs/BoundingBoxes.h>

using namespace std;
using namespace cv;
using namespace pcl;

/*
OpenCV Coordinate System
row == height == Point.y
col == width  == Point.x
*/

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frame (new pcl::PointCloud<pcl::PointXYZ>);
#define PI 3.1415926
bool find_object  = false;
bool adjust_robot = true;
bool in_position  = false;
int camera_width = 0;
int camera_height = 0;
int center_x = 0;
int center_y = 0;
int object_x = 0;
int object_y = 0;
std::string RGB_CAMERA_BASE;
std::string ARM_BASE;



ros::Subscriber nav_sub;
ros::Subscriber darknet_sub;
ros::Subscriber camera_info;
ros::Subscriber pcl_sub;
ros::Subscriber speech_sub;

ros::Publisher  turn_robot;
ros::Publisher  obj_pub;





void adjustRobotOrientation (float goal_angle)
{
    while (adjust_robot)
    {
        if ()
    }


}

void cameraInfoCallback (const sensor_msgs::CameraInfo::ConstPtr& camera_info)
{
    if (camera_height == 0 || camera_width == 0)
    {
        ROS_INFO ("Receive Camera Info");
        camera_height = camera_info->height;
        camera_width  = camera_info->width;
        center_x = int (camera_height / 2);
        center_y = int (camera_width / 2);
    }
}

std::string object_name;
void speechCallback (const std_msgs::String::ConstPtr& msg)
{
    //从语音节点处获取到要取的物品的名称
    object_name = msg->data;
}

void navCallback (const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "in_position")
    {
        
        //此处是接受到导航节点的消息
        //在到达指定地点之后导航节点会发布消息，之后开始转动机器人并寻找物体
        /*
        @TODO
        此处根据具体需求修改导航节点传过来的消息可以表示到达了不同的地点
        */
        //没有找到物体时会一直转动机器人，同时运行darknet进行识别
        ROS_INFO ("Arrived at the nav point\n");
        ROS_INFO ("Now turn robot to find graspable object\n");
        //每次转动60度来寻找物体
        geometry_msgs::Twist vel;
        double rate = 50;
        ros::Rate loopRate (rate);

        //设定转动角速度
        float angular_speed = 0.5;
        //每次转动目标角度是60度
        float goal_angle    = PI / 3;
        //每次转动到目标角度需要的时间
        float turn_duration = goal_angle / angular_speed; 
        //需要发布多少次指令来实现，这一项与我们上面设定的频率有关
        int ticks = int (turn_duration * rate);   
        vel.linear.x  = 0;
        vel.linear.y  = 0;
        vel.linear.z  = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = angular_speed;

        while (!find_object)
        {
            for (int i=0; i<ticks; i++)
            {
                turn_robot.publish (vel);
                //命令机器人休眠一个Rate的时间
                loopRate.sleep();
            }
            vel.angular.z = 0;
        }
        //当find_object变为true即找到了物体的时候
        //向发送一个空的Twist消息使机器人保持静止
        turn_robot.publish(geometry_msgs::Twist());
    }
}

void darknetCallback (darknet_ros_msgs::BoundingBoxes msg)
{

    /*
    @TODO
    是否需要把程序修改为开始微调机器人之后
    每次转动完机器人之后，拍一张照发送给darknet识别
    而不是使用实时识别
    */

    //找到要抓取物体的B-box
    int idx = 0;
    if (msg.boundingBoxes.size() > 1)
    {
        for (int i = 0; i<msg.boundingBoxes.size(); i++)
        {
            if (msg.boundingBoxes[i].Class == object_name)
            {
                //当找到了带抓取物体的时候
                //记录这个物体对应的id
                //修改find_object使机器人停止转动
                idx = i;
                printf ("Detected %d Bounding_Boxes\n",object_name);
                printf ("\t Bounding_Boxes %d=[Xmin Ymin Xmax Ymax]=[%d %d %d %d]\n", object_name,
				msg.boundingBoxes[i].xmin,
				msg.boundingBoxes[i].ymin,
				msg.boundingBoxes[i].xmax,
				msg.boundingBoxes[i].ymax);
                object_x = int ((msg.boundingBoxes[i].xmin + msg.boundingBoxes[i].xmax) / 2);
                object_y = int ((msg.boundingBoxes[i].ymin + msg.boundingBoxes[i].ymax) / 2);
                find_object = true;
            }
        }
    }
}

void pclCallback (sensor_msgs::PointCloud2 msg)
{
    if (in_position == true)
    {
        /*
        @TODO
        在完成机器人搭建并修改URDF之后需要使用tf变换将原本在相机坐标系下的点云
        变换到机械臂底座的坐标系中
        */
        try
        {
            pListener->lookupTransform(ARM_BASE, RGB_CAMERA_BASE, CurrTF);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR ("%s", ex.what());
            return;
            ros::Duration(1.0).sleep();
        }
        tf::Vector3 P = currTF.getOrigin();
        tf::Matrix3x3 R = currTF.getBasis();
        

    }
}

void getObjectPosition()
{
    //从tf变换完之后的点云中获取到待抓取物体在机械臂坐标系中的位置
    int idx[9] = {(object_y-1)*camera_width+object_x-1, (object_y-1)*camera_width+object_x, (object_y-1)*camera_width+object_x+1,
                      object_y*camera_width+object_x-1, object_y*camera_width+object_x, object_y*camera_width+object_x+1,
                      (object_y+1)*camera_width+object_x-1, (object_y+1)*camera_width+object_x, (object_y+1)*camera_width+object_x+1};
}

int main(int argc, char** argv)
{
    ROS_INFO ("\tWaiting for signal\t");
    ROS_INFO ("\tinitial ROS node\t");
    ros::init (argc, argv, "find_object");
    ros::NodeHandle nh;

    //publisher & subscriber
    nav_sub     = nh.subscribe ("/nav2img", 1, navCallback);
    darknet_sub = nh.subscribe ("/bounding_boxes", 1, darknetCallback);
    camera_info = nh.subscribe ("/camera/depth/camera_info", 1, cameraInfoCallback);
    pcl_sub     = nh.subscribe ("/camera/depth/points", 1, pclCallback);
    speech_pub  = nh.subscribe ("/speech2img", 1, speechCallback);

    turn_robot  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    //到达合适位置之后，发布一个物体在机械臂坐标系下的（X，Y，Z）位置信息
    obj_pub     = nh.advertise<std_msgs::String>("/grasp_object", 1);




}