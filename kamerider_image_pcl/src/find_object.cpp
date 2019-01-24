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
#include <actionlib/client/simple_action_client.h>

//user-defined ROS message type
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <kamerider_image_pcl/object_position.h>
#include <kamerider_navigation/turn_robotAction.h>


using namespace std;
using namespace cv;
using namespace pcl;

typedef actionlib::SimpleActionClient<kamerider_navigation::turn_robotAction> Client;
Client client("turn_robot", true);
/*
OpenCV Coordinate System
row == height == Point.y
col == width  == Point.x
*/
unsigned char floatBuffer[4];
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frame (new pcl::PointCloud<pcl::PointXYZ>);
//用来储存使用tf变换到机械臂坐标下的点云数据
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_arm   (new pcl::PointCloud<pcl::PointXYZ>);
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
std::string DEPTH_BASE     = "/camera_link";
std::string ARM_BASE       = "/arm_base_link";
std::string TURTLEBOT_BASE = "/base_link";

tf::StampedTransform currTF;//监听到的tf变换
tf::TransformListener*pListener;//监听tf变换的设备指针,具体定义在main函数ros::init之后了

//------------------------------
//| ROS Subscriber & Publisher |
//------------------------------
ros::Subscriber nav_sub;
ros::Subscriber darknet_sub;
ros::Subscriber camera_info;
ros::Subscriber pcl_sub;
ros::Subscriber speech_sub;

ros::Publisher  turn_robot;
ros::Publisher  obj_pub;
ros::Publisher  pcl_pub;

void doneCallback (const actionlib::SimpleClientGoalState& state,
                   const kamerider_navigation::turn_robotResultConstPtr& result)
{
    ROS_INFO ("turn_robot %f finished, start detecting graspable object", result->final_angle);
}

void activeCallback ()
{
    ROS_INFO ("Goal setted, start turn_robot action");
}

void feedbackCallback (const kamerider_navigation::turn_robotFeedbackConstPtr& feedback)
{
    ROS_INFO ("Current angle: %f", feedback->current_angle);
}

void adjustRobotOrientation ()
{
    while (adjust_robot)
    {
        //当没有达到要求的位置的时候发布Twist消息来轻微转到机器人
        //每次转动10度来调整机器人

        //物体中心在屏幕中心左侧的时候需要逆时针转动机器人
        if (object_x < center_x)
        {
            //目标角度是10度
            kamerider_navigation::turn_robotGoal goal;
            goal.goal_angle = PI/18;
            client.sendGoal (goal, &doneCallback, &activeCallback, &feedbackCallback);
            //每次转动之后判断一次物体与视野中心相对位置，如果已经满足要求则停止转动
            if (fabs(center_x - object_x) < 10)
            {
                ROS_INFO ("Aiming at the Graspable object");
                ROS_INFO ("Stop Adjusting Robot");
                adjust_robot = false;
                in_position  = true;
                break;
            }
 
        }

        //物体中心在屏幕中心右侧的时候需要顺时针转动机器人
        if (object_x > center_x)
        {
            //目标角度是-10度
            //目标角度是10度
            kamerider_navigation::turn_robotGoal goal;
            goal.goal_angle = (-PI/18);
            client.sendGoal (goal, &doneCallback, &activeCallback, &feedbackCallback);
            //每次转动之后判断一次物体与视野中心相对位置，如果已经满足要求则停止转动
            if (fabs(center_x - object_x) < 10)
            {
                ROS_INFO ("Aiming at the Graspable object");
                ROS_INFO ("Stop Adjusting Robot");
                adjust_robot = false;
                in_position  = true;
                break;
            } 
        }

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
        while (!find_object)
        {
            kamerider_navigation::turn_robotGoal goal;
            goal.goal_angle = PI/3;
            client.sendGoal (goal, &doneCallback, &activeCallback, &feedbackCallback);
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
    if (msg.bounding_boxes.size() > 1)
    {
        for (int i = 0; i<msg.bounding_boxes.size(); i++)
        {
            if (msg.bounding_boxes[i].Class == object_name)
            {
                //当找到了带抓取物体的时候
                //记录这个物体对应的id
                //修改find_object使机器人停止转动
                idx = i;
                printf ("Detected %s Bounding_Boxes\n",object_name);
                printf ("\t Bounding_Boxes %s=[Xmin Ymin Xmax Ymax]=[%d %d %d %d]\n", object_name,
				msg.bounding_boxes[i].xmin,
				msg.bounding_boxes[i].ymin,
				msg.bounding_boxes[i].xmax,
				msg.bounding_boxes[i].ymax);
                object_x = int ((msg.bounding_boxes[i].xmin + msg.bounding_boxes[i].xmax) / 2);
                object_y = int ((msg.bounding_boxes[i].ymin + msg.bounding_boxes[i].ymax) / 2);
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
            pListener->lookupTransform(ARM_BASE, DEPTH_BASE, ros::Time(0), currTF);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR ("%s", ex.what());
            return;
            ros::Duration(1.0).sleep();
        }
        tf::Vector3 P = currTF.getOrigin();
        tf::Matrix3x3 R = currTF.getBasis();
        
        //对点云数据进行tf变换
        sensor_msgs::PointCloud2 oMsg;
        cloud_arm->width = msg.width;
        cloud_arm->height = msg.height;
        cloud_arm->points.resize (msg.height * msg.width);

        for (size_t i=0; i<cloud_arm->points.size(); i++)
        {
            //把已有点云消息中的X,Y,Z提取出来
            floatBuffer[0]=msg.data[i*16+0];
            floatBuffer[1]=msg.data[i*16+1];
            floatBuffer[2]=msg.data[i*16+2];
            floatBuffer[3]=msg.data[i*16+3];
            double X=*((float*)floatBuffer);
            
            floatBuffer[0]=msg.data[i*16+4];
            floatBuffer[1]=msg.data[i*16+5];
            floatBuffer[2]=msg.data[i*16+6];
            floatBuffer[3]=msg.data[i*16+7];
            double Y=*((float*)floatBuffer);
            
            floatBuffer[0]=msg.data[i*16+8];
            floatBuffer[1]=msg.data[i*16+9];
            floatBuffer[2]=msg.data[i*16+10];
            floatBuffer[3]=msg.data[i*16+11];
            double Z=*((float*)floatBuffer);

            cloud_arm->points[i].x = R[0][0]*X+R[0][1]*Y+R[0][2]*Z+P[0];
            cloud_arm->points[i].y = R[1][0]*X+R[1][1]*Y+R[1][2]*Z+P[1];
            cloud_arm->points[i].z = R[2][0]*X+R[2][1]*Y+R[2][2]*Z+P[2];
        }
        pcl::toROSMsg (*cloud_arm, oMsg);
        oMsg.header.frame_id = "arm_base";
        pcl_pub.publish (oMsg);
    }
}

void getObjectPosition()
{
    //从tf变换完之后的点云中获取到待抓取物体在机械臂坐标系中的位置
    //然后由obj_pub发布出去
    int idx[9] = {(object_y-1)*camera_width+object_x-1, (object_y-1)*camera_width+object_x, (object_y-1)*camera_width+object_x+1,
                      object_y*camera_width+object_x-1, object_y*camera_width+object_x, object_y*camera_width+object_x+1,
                      (object_y+1)*camera_width+object_x-1, (object_y+1)*camera_width+object_x, (object_y+1)*camera_width+object_x+1};
    if (cloud_arm->empty())
    {
        ROS_INFO ("Waiting For PCL Transform");
        sleep (2);
    }
    else
    {
        //遍历由darknet产生的bounding_box确定的物体中心点周围的9个点
        //然后使用其中的非Nan点的均值来代替中心点
        float x = 0;
        float y = 0;
        float z = 0;
        int num = 0;
        for (int i=0; i<9 ;i++)
        {
            if ((!cloud_arm->points[idx[i]].x) || (!cloud_arm->points[idx[i]].y) || (!cloud_arm->points[idx[i]].z))
            {
                x += cloud_arm->points[idx[i]].x;
                y += cloud_arm->points[idx[i]].y;
                z += cloud_arm->points[idx[i]].z;
                num ++;
            }
        }
        kamerider_image_pcl::object_position pos;
        pos.x = x/num;
        pos.y = y/num;
        pos.z = z/num;
        obj_pub.publish (pos);
    }
}

int main(int argc, char** argv)
{
    ROS_INFO ("Waiting for action server");
    client.waitForServer();
    ROS_INFO ("Action server started");
    ROS_INFO ("\tWaiting for signal\t");
    ROS_INFO ("\tinitial ROS node\t");
    ros::init (argc, argv, "find_object");
    ros::NodeHandle nh;

    //publisher & subscriber
    nav_sub     = nh.subscribe ("/nav2img", 1, navCallback);
    darknet_sub = nh.subscribe ("/bounding_boxes", 1, darknetCallback);
    camera_info = nh.subscribe ("/camera/depth/camera_info", 1, cameraInfoCallback);
    pcl_sub     = nh.subscribe ("/camera/depth/points", 1, pclCallback);
    speech_sub  = nh.subscribe ("/speech2img", 1, speechCallback);

    turn_robot  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pcl_pub     = nh.advertise<sensor_msgs::PointCloud2>("/arm_base_pcl", 1);
    //到达合适位置之后，发布一个物体在机械臂坐标系下的（X，Y，Z）位置信息
    obj_pub     = nh.advertise<kamerider_image_pcl::object_position>("/object_position", 1);
    
    tf::TransformListener Listener;
	pListener = &Listener;
    ros::spin();
}