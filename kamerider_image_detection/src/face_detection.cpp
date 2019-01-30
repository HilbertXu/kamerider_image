/*
Date: 2019/1/24
Author: Xu Yucheng
E-mail: hilbertxu@outlook.com
Abstract: Code for face detection in ros by using Dlib
*/

/*NOTE:
OpenCV frames
row == heigh == Point.y
col == width == Point.x
Mat::at(Point(x, y)) == Mat::at(y,x)
*/

// ROS headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Dlib headers
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/opencv.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>

// Opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// C++
#include <iostream>
#include <vector>
#include <stdlib.h>

// kamerider_image_msgs
#include <kamerider_image_msgs/FaceDetection.h>
#include <kamerider_image_msgs/BoundingBox.h>


using namespace std;
using namespace dlib;

class face_detection
{
private:

    //从参数服务器中获取到的图像话题名称
    std::string sub_image_topic_name;
    std::string pub_face_detected_topic_name;
    std::string pretrained_dataset;

    ros::Subscriber sub_image;
    ros::Publisher  pub_result;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;
        face_detection::face_detect(img, face_detection::pretrained_dataset);

    }

    cv::Mat my_resize_image(cv::Mat pSrc, double rScale, double cScale)
    {
        cv::Size pSize = cv::Size(pSrc.cols*cScale, pSrc.rows*rScale);
        cv::Mat pRes = cv::Mat (pSize, CV_32S);
        cv::resize(pSrc, pRes, pSize);
        return pRes;
    }

    void publishMessage(ros::Publisher &publisher, const kamerider_image_msgs::FaceDetection &message)
    {
        ROS_INFO("Publish message");
        kamerider_image_msgs::FaceDetection face_detection_msg;
        face_detection_msg.header   = message.header;
        face_detection_msg.face_num = message.face_num;
        for (int i = 0; i< sizeof(message.bounding_boxes); i++)
        {
            face_detection_msg.bounding_boxes[i].xmax = message.bounding_boxes[i].xmax;
            face_detection_msg.bounding_boxes[i].xmin = message.bounding_boxes[i].xmin;
            face_detection_msg.bounding_boxes[i].ymax = message.bounding_boxes[i].ymax;
            face_detection_msg.bounding_boxes[i].ymin = message.bounding_boxes[i].ymin;
        }
        
        publisher.publish(face_detection_msg);
    }
    
    //函数接受一个Mat矩阵类型的参数，以及一个字符串参数指向需要加载的数据集
    void face_detect(cv::Mat pSrc, std::string dataset)
    {
        cv::Mat scaled_img;
        //首先检测图像尺寸，若尺寸过大，则需要对图像进行压缩
        if (pSrc.cols >= 640 || pSrc.rows >= 480)
        {
            double rScale = 480 / (pSrc.rows);
            double cScale = 640 / (pSrc.cols);
            scaled_img = face_detection::my_resize_image(pSrc, rScale, cScale);
        }
        else
        {
            scaled_img = face_detection::my_resize_image(pSrc, 1, 1);
        }

        dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
        dlib::shape_predictor sp;
        dlib::deserialize(dataset) >> sp;
        dlib::image_window window, window_faces;

        //将opencv下mat格式的图片转换为Dlib格式的图片
        dlib::array2d<dlib::rgb_pixel> img;
        dlib::assign_image(img, dlib::cv_image<dlib::bgr_pixel>(scaled_img));
        //放大图像来检测小的人脸部分
        dlib::pyramid_up(img);

        //使用detectoe来检测照片中可能存在的人脸，并使用bounding boxes框出来
        std::vector<dlib::rectangle> rects = detector(img);
        std::cout << "Number of faces detected is: " << rects.size() << std::endl;

        //使用shape_predictor来确定每个人脸的bounding boxes在图片中的位置
        std::vector<dlib::full_object_detection> shapes;

        //生成一个自定义消息类型的对象，来储存识别的结果
        kamerider_image_msgs::FaceDetection DetectResult;
        DetectResult.face_num = rects.size();
        for (unsigned long j = 0; j< rects.size(); j++)
        {
            dlib::full_object_detection shape = sp(img, rects[j]);
            cout << "number of parts: " << shape.num_parts() << endl;
            cout << "pixel position of first part:  " << shape.part(0) << endl;
            cout << "pixel position of second part: " << shape.part(1) << endl;
            shapes.push_back(shape);

            DetectResult.bounding_boxes[j].xmin = shape.part(0).x();
            DetectResult.bounding_boxes[j].ymin = shape.part(0).y();
            DetectResult.bounding_boxes[j].xmax = shape.part(1).x();
            DetectResult.bounding_boxes[j].ymax = shape.part(1).y();
        }
        face_detection::publishMessage(pub_result, DetectResult);
        window.clear_overlay();
        window.set_image(img);
        window.add_overlay(dlib::render_face_detections(shapes));
    }
public:
    int run (int argc, char** argv)
    {
        //初始化ros节点
        ros::init(argc, argv, "face_detection");
        ros::NodeHandle nh;

        nh.param<std::string>("sub_image_raw_topic_name",     sub_image_topic_name,         "/image_raw");
        nh.param<std::string>("pub_face_detected_topic_name", pub_face_detected_topic_name, "/kamerider_image_detection/face_detect_result");
        nh.param<std::string>("pretrained_dataset",           pretrained_dataset,           "/home/kamerider/catkin_ws/src/kamerider_image/kamerider_image_msgs/shape_predictor_68_face_landmarks.dat");

        //定义订阅器和发布器
        sub_image  = nh.subscribe(sub_image_topic_name, 1, &face_detection::imageCallback, this);
        pub_result = nh.advertise<kamerider_image_msgs::FaceDetection>(pub_face_detected_topic_name, 10);
        ros::spin();
    }
};

int main(int argc, char** argv)
{
    face_detection detector;
    return detector.run(argc, argv);
}
