/*
    Date: 2019/03/13
    Author: Xu Yucheng & Zeng Qingyi
    Abstract: object detection with yolo, 3D-position predict with PCL
*/
#define PI 3.1415926

// common headers
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <cmath>
#include <sstream>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>

// user-defined ROS message type
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <kamerider_image_msgs/ObjectPosition.h>
#include <kamerider_navigation/turn_robotAction.h>

using namespace std;
using namespace pcl;
using namespace cv;

typedef actionlib::SimpleActionClient<kamerider_navigation::turn_robotAction> Client;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_astra (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base (new pcl::PointCloud<pcl::PointXYZ>);
unsigned char floatBuffer[4];
    
bool find_object = true;
bool adjust_robot = false;
bool in_position = false;
int camera_width = 640;
int camera_height = 480;
int center_x = 0;
int center_y = 0;
int object_x = 0;
int object_y = 0;
int origin_index = 0;
int valid_index = 0;


// turn robot server callback functions
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

void on_mouse (int event, int x, int y, int flags, void* param)
{
    Mat *im = reinterpret_cast<Mat*>(param);

    switch(event)
    {
        case CV_EVENT_LBUTTONDOWN:
            //左键按下响应：输出并保存点击处的坐标
            int row = y;
            int col = x;
            int idx = row*camera_width + col;
            int new_idx = 0;
            //if(isnan(cloud_astra.points[idx].x) || isnan(cloud_astra.points[idx].y) || isnan(cloud_astra.points[idx].z))
            //{
            //    new_idx = FindNearValid(idx);
            //}
            //从原始点云中移除Nans
            //std::vector<int> mapping;
            //removeNaNFromPointCloud(cloud_astra, cloud_astra, mapping);
            std::cout << "at(" << x << "," << y << ")" <<std::endl;
            object_x = x;
            object_y = y;
            std::cout << "3D position(" << cloud_astra->points[new_idx].x <<","
                                        << cloud_astra->points[new_idx].y << "," 
                                        << cloud_astra->points[new_idx].z << ")" <<std::endl;     
            break;

    }
}

class object_detection
{
private:
    // ROS parameters
    std::string sub_bbox_topic_name;
    std::string sub_pcl_topic_name;
    std::string sub_image_raw_topic_name;
    std::string sub_cam_info_topic_name;
    std::string sub_nav_topic_name;
    std::string pub_object_pos_topic_name;
    std::string pub_tf_pcl_topic_name;

    // ROS subscriber & publisher
    ros::Subscriber bbox_sub;
    ros::Subscriber pcl_sub;
    ros::Subscriber cam_sub;
    ros::Subscriber nav_sub;
    ros::Subscriber img_sub;
    ros::Publisher obj_pub;
    ros::Publisher pcl_pub;

    // parameters
    std::string CAMERA_DEPTH_FRAME = "astra_depth_frame";
    std::string TURTLEBOT_BASE_FRAME = "base_link";

    // @TODO 
    // 测试时使用orange juice这个名称，在正式使用中需要修改为接受语音节点发过来的识别结果
    std::string object_name = "Orange Juice";
    
    int find_near_valid(int idx)
    {   
        double temp_min = 999999;
        int return_idx = idx;
        int obj_row = idx/camera_width;
        int obj_col = idx%camera_width;

        for (int row=0; row<camera_height; row++)
        {
            for (int col=0; col<camera_width; col++)
            {
                if (!isnan(cloud_astra->points[row*camera_width+col].x) &&
                    !isnan(cloud_astra->points[row*camera_width+col].y) &&
                    !isnan(cloud_astra->points[row*camera_width+col].z))
                {
                    double dis = (row-obj_row)*(row-obj_row) + (col-obj_col)*(col-obj_col);
                    if (dis < temp_min)
                    {
                        return_idx = row*camera_width + col;
                        temp_min = dis;
                    }
                }
            }
        }
        valid_index = return_idx;
        return return_idx;
    }

    // @TODO 完成机器人朝向的微调代码
    void adjust_robot_orientation();

    // @TODO 完成导航部分的通信
    void navCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::cout << "This function has not been defined" << std::endl;
        if (msg->data == "in_grasp_position")
        {
            in_position = true;
        }
    }

    void caminfoCallback (const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        if (camera_height == 0 || camera_width == 0)
        {
            ROS_INFO ("Received Camera Info");
            camera_height = msg->height;
            camera_width = msg->width;
            center_x = int (camera_height / 2);
            center_y = int (camera_width / 2);
        }
    }

    void imageCallback (const sensor_msgs::ImageConstPtr& msg)
    {
        if (cloud_astra->empty())
        {
            int ori_row = origin_index / camera_width;
            int ori_col = origin_index % camera_width;
            int val_row = valid_index / camera_width;
            int val_col = valid_index % camera_width;
            cout << origin_index << " " << valid_index << endl;
            cout << ori_row << " " << ori_col << " " << val_row << " " << val_col << endl;
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = cv_ptr->image;
            Point ori(ori_col, ori_row);
            Point val(val_col, val_row);

            circle(img, ori, 3, Scalar(0,0,255), -1);
            circle(img, val, 3, Scalar(255,0,0), -1);

            cv::imshow("object_detection", img);
            cv::waitKey(1);

            cv::setMouseCallback("object_detection", on_mouse, reinterpret_cast<void*> (&img));
        }
    }
    void pclCallback(sensor_msgs::PointCloud2 msg)
    {
        // 机器人到达抓取位置之后开始监听点云变换
        if (in_position == true)
        {
            // 储存原始点云数据
            pcl::fromROSMsg (msg, *cloud_astra);
        }
    }

    void get_object_position()
    {
        cv::Mat cam_pos = cv::Mat::ones(3,1, CV_32FC1);
        cv::Mat R = cv::Mat::ones(3,3, CV_32FC1);
        cv::Mat T = cv::Mat::ones(3,1, CV_32FC1);
        cv::Mat result;
        R.at<float>(0,0) = 0.50000653951382756;
        R.at<float>(0,1) = 0;
        R.at<float>(0,2) = 0.86602162816144901;
        R.at<float>(1,0) = 0;
        R.at<float>(1,1) = 1;
        R.at<float>(1,2) = 0;
        R.at<float>(2,0) = -0.86602162816144901;
        R.at<float>(2,1) = 0;
        R.at<float>(2,2) = 0.50000653951382756;
        T.at<float>(0,0) = -0.013639999999999999;
        T.at<float>(0,1) = 0.025000000000000005;
        T.at<float>(0,2) =  1.0696000000000001;

        if (cloud_astra->empty())
        {
            ROS_INFO ("Waiting for pcl transform");
            sleep (2);
        }
        else
        {
            origin_index = object_y * camera_width + object_x;
            kamerider_image_msgs::ObjectPosition pos;
            if (isnan(cloud_astra->points[origin_index].x) || 
                isnan(cloud_astra->points[origin_index].y) ||
                isnan(cloud_astra->points[origin_index].z))
            {  
                int new_index = find_near_valid(origin_index);
                cam_pos.at<float>(0,0) = cloud_astra->points[new_index].x;
                cam_pos.at<float>(0,1) = cloud_astra->points[new_index].y;
                cam_pos.at<float>(0,2) = cloud_astra->points[new_index].z;
                result = R*cam_pos + T;
            }
            else
            {
                cam_pos.at<float>(0,0) = cloud_astra->points[origin_index].x;
                cam_pos.at<float>(0,0) = cloud_astra->points[origin_index].y;
                cam_pos.at<float>(0,0) = cloud_astra->points[origin_index].z;
                result = R*cam_pos + T;
            }
            pos.header.frame_id = "/base_link";
            pos.x = result.at<float>(0,0);
            pos.y = result.at<float>(0,1);
            pos.z = result.at<float>(0,2);
            obj_pub.publish (pos);
        }
    }

public:
    int run (int argc, char** argv)
    {
        ROS_INFO("----------INIT----------");
        ros::init (argc, argv, "object_detection_on_mouse");
        ros::NodeHandle nh;
        ROS_INFO("----Waiting for image----");

        nh.param<std::string>("sub_bbox_topic_name",       sub_bbox_topic_name,       "/darknet_ros/bounding_boxes");
        nh.param<std::string>("sub_pcl_topic_name",        sub_pcl_topic_name,        "/camera/depth/points");
        nh.param<std::string>("sub_cam_info_topic_name",   sub_cam_info_topic_name,   "/camera/depth/info");
        nh.param<std::string>("sub_nav_topic_name",        sub_nav_topic_name,        "/kamerider_navigation/nav_pub");
        nh.param<std::string>("sub_image_raw_topic_name",  sub_image_raw_topic_name,  "/camera/rgb/image_raw");
        nh.param<std::string>("pub_object_pos_topic_name", pub_object_pos_topic_name, "/kamerider_image/object_position_astra");
        nh.param<std::string>("pub_tf_pcl_topic_name",     pub_tf_pcl_topic_name,     "/base_link_pcl");

        pcl_sub = nh.subscribe(sub_pcl_topic_name, 1, &object_detection::pclCallback, this);
        // bbox_sub = nh.subscribe(sub_bbox_topic_name, 1, &object_detection::darknetCallback, this);
        cam_sub = nh.subscribe(sub_cam_info_topic_name, 1, &object_detection::caminfoCallback, this);
        nav_sub = nh.subscribe(sub_nav_topic_name, 1, &object_detection::navCallback, this);
        img_sub = nh.subscribe(sub_image_raw_topic_name, 1, &object_detection::imageCallback, this);
        obj_pub = nh.advertise<kamerider_image_msgs::ObjectPosition>(pub_object_pos_topic_name, 1);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(pub_tf_pcl_topic_name, 1);
        
        std::cout << "Receving message from topics: " << std::endl;
        std::cout << "--------------------------" << std::endl;
        std::cout << "\t" << sub_bbox_topic_name << std::endl;
        std::cout << "\t" << sub_pcl_topic_name << std::endl;
        std::cout << "\t" << sub_cam_info_topic_name << std::endl;
        std::cout << "\t" << sub_nav_topic_name << std::endl;
        std::cout << "--------------------------" << std::endl;
        std::cout << "Publishing message to topics: " << std::endl;
        std::cout << "--------------------------" << std::endl;
        std::cout << "\t" << pub_object_pos_topic_name << std::endl;
        std::cout << "\t" << pub_tf_pcl_topic_name << std::endl;
        std::cout << "--------------------------" << std::endl;

        ros::spin();
    }

};

int main(int argc, char** argv)
{
    object_detection detector;
    return detector.run(argc, argv);
}

