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
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <darknet_ros_msgs/BoundingBoxes.h>

using namespace std;
using namespace cv;
using namespace pcl;


#define PI 3.1415926
bool find_object = false;


ros::Subscriber nav_sub;
ros::Subscriber darknet_sub;
ros::Publisher  turn_robot;
ros::Publisher  obj_pub;
ros::Publisher  speech_pub;

void adjustRobotOrientation (float goal_angle);

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
        /*
        此处是接受到导航节点的消息
        在到达指定地点之后导航节点会发布消息，之后开始转动机器人并寻找物体

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
                find_object = true;
            }
        }
    }
}

void pclCallback (sensor_msgs::PointCloud2 msg);