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

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frame (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_arm (new pcl::PointCloud<pcl::PointXYZ>);
unsigned char floatBuffer[4];
    
bool find_object = false;
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
                if (!isnan(cloud_arm->points[row*camera_width+col].x) &&
                    !isnan(cloud_arm->points[row*camera_width+col].y) &&
                    !isnan(cloud_arm->points[row*camera_width+col].z))
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
        if (find_object && !cloud_arm->empty())
        {
            int ori_row = origin_index / camera_width;
            int ori_col = origin_index % camera_width;
            int val_row = valid_index / camera_width;
            int val_col = valid_index & camera_width;
            cout << ori_row << " " << ori_col << " " << val_row << " " << val_col << endl;
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = cv_ptr->image;
            Point ori(ori_col, ori_row);
            Point val(val_col, val_row);

            circle(img, ori, 3, Scalar(0,0,255), -1);
            circle(img, val, 3, Scalar(255,0,0), -1);

            cv::imshow("object_detection", img);
            cv::waitKey(10);
        }
    }
    
    void darknetCallback (darknet_ros_msgs::BoundingBoxes msg)
    {
        if (msg.bounding_boxes.size() > 0)
        {
            
            for (int i=0; i<msg.bounding_boxes.size(); i++)
            {
                if (msg.bounding_boxes[i].Class == object_name)
                {
                    // printf ("Detected %s Bounding_Boxes\n",object_name.c_str());
                    // printf ("Bounding_Boxes %s=[Xmin Ymin Xmax Ymax]=[%d %d %d %d]\n", object_name.c_str(),
                    // msg.bounding_boxes[i].xmin,
                    // msg.bounding_boxes[i].ymin,
                    // msg.bounding_boxes[i].xmax,
                    // msg.bounding_boxes[i].ymax);
                    object_x = int ((msg.bounding_boxes[i].xmin + msg.bounding_boxes[i].xmax) / 2);
                    object_y = int ((msg.bounding_boxes[i].ymin + msg.bounding_boxes[i].ymax) / 2);
                    //printf ("The center pixel of %s is: [row, col] = [%d, %d]\n", object_name.c_str(), object_x, object_y);
                    find_object = true;
                }
            }
        }
        get_object_position();
    }

    void pclCallback(sensor_msgs::PointCloud2 msg)
    {
        // 机器人到达抓取位置之后开始监听点云变换
        if (in_position == true)
        {
            // TF listener
            tf::StampedTransform currTF;
            tf::TransformListener pListener;
            try
            {
                cout << "Looking for transform" << endl;
                pListener.lookupTransform("astra_depth_frame", "/base_link", ros::Time(0), currTF);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR ("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            tf::Vector3 T = currTF.getOrigin();
            tf::Matrix3x3 R = currTF.getBasis();
            printf("%.17g\t%.17g\t%.17g\t%.17g\n" \
						"%.17g\t%.17g\t%.17g\t%.17g\n" \
						"%.17g\t%.17g\t%.17g\t%.17g\n" \
						"0\t0\t0\t1\n",
						R[0][0],R[0][1],R[0][2],T[0],
						R[1][0],R[1][1],R[1][2],T[1],
						R[2][0],R[2][1],R[2][2],T[2]);
		    cout<<"--------------------------------------"<<endl;

            // 对点云数据进行tf变换，由相机坐标系转换到机器人底盘坐标系
            sensor_msgs::PointCloud2 oMsg;
            cloud_arm->width = msg.width;
            cloud_arm->height = msg.height;
            cout << msg.width << " " << msg.height << endl;
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

                cloud_arm->points[i].x = R[0][0]*X+R[0][1]*Y+R[0][2]*Z+T[0];
                cloud_arm->points[i].y = R[1][0]*X+R[1][1]*Y+R[1][2]*Z+T[1];
                cloud_arm->points[i].z = R[2][0]*X+R[2][1]*Y+R[2][2]*Z+T[2];
            }
            pcl::toROSMsg (*cloud_arm, oMsg);
            oMsg.header.frame_id = "base_link";
            pcl_pub.publish (oMsg);
        }
    }

    void get_object_position()
    {
        if (cloud_arm->empty())
        {
            ROS_INFO ("Waiting for pcl transform");
            sleep (2);
        }
        else
        {
            origin_index = object_y * camera_width + object_x;
            cout << "origin_index: "<< origin_index << endl;
            kamerider_image_msgs::ObjectPosition pos;
            if (isnan(cloud_arm->points[origin_index].x) || 
                isnan(cloud_arm->points[origin_index].y) ||
                isnan(cloud_arm->points[origin_index].z))
            {  
                int new_index = find_near_valid(origin_index);
                pos.x = cloud_arm->points[new_index].x;
                pos.y = cloud_arm->points[new_index].y;
                pos.z = cloud_arm->points[new_index].z;
            }
            else
            {
                pos.x = cloud_arm->points[origin_index].x;
                pos.y = cloud_arm->points[origin_index].y;
                pos.z = cloud_arm->points[origin_index].z;   
            }
            
            obj_pub.publish (pos);
        }
    }

public:
    int run (int argc, char** argv)
    {
        ROS_INFO("----------INIT----------");
        ros::init (argc, argv, "object_detection");
        ros::NodeHandle nh;
        ROS_INFO("----Waiting for image----");

        nh.param<std::string>("sub_bbox_topic_name",       sub_bbox_topic_name,       "/darknet_ros/bounding_boxes");
        nh.param<std::string>("sub_pcl_topic_name",        sub_pcl_topic_name,        "/camera/depth/points");
        nh.param<std::string>("sub_cam_info_topic_name",   sub_cam_info_topic_name,   "/camera/depth/info");
        nh.param<std::string>("sub_nav_topic_name",        sub_nav_topic_name,        "/kamerider_navigation/nav_pub");
        nh.param<std::string>("sub_image_raw_topic_name",  sub_image_raw_topic_name,  "/camera/rgb/image_raw");
        nh.param<std::string>("pub_object_pos_topic_name", pub_object_pos_topic_name, "/kamerider_image/object_position");
        nh.param<std::string>("pub_tf_pcl_topic_name",     pub_tf_pcl_topic_name,     "/base_link_pcl");

        pcl_sub = nh.subscribe(sub_pcl_topic_name, 1, &object_detection::pclCallback, this);
        bbox_sub = nh.subscribe(sub_bbox_topic_name, 1, &object_detection::darknetCallback, this);
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

