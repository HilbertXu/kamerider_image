/*
!!!!CODE FOR TEST
Date: 2019/1/31
Author: Xu Yucheng
Abstract: A pure dlib face detection demo
*/

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
#include "kamerider_image_detection/timestramp.h"


using namespace std;
using namespace dlib;

cv::Mat MyResizeImage(cv::Mat pSrc, double dScale)
{
	cv::Size sSize = cv::Size(pSrc.cols*dScale, pSrc.rows*dScale);
	cv::Mat pDes = cv::Mat(sSize, CV_32S);
	resize(pSrc, pDes, sSize);
	return pDes;
}
 
// ----------------------------------------------------------------------------------------
 
int main(int argc, char** argv)
{  
    try
    {
        frontal_face_detector detector = get_frontal_face_detector();
        shape_predictor sp;
        deserialize("/home/kamerider/catkin_ws/src/kamerider_image/kamerider_image_detection/shape_predictor_68_face_landmarks.dat") >> sp;
 
 
        dlib::image_window window;
		while (1)
		{
			timestramp tp;
			// Grab a frame
			cv::Mat temp;
			temp = cv::imread("/home/kamerider/Pictures/test.png");
			//cv::cvtColor(temp,temp,CV_BGR2GRAY);
            //cv::Size pSize = cv::Size(640, 480);
            //cv::resize(temp, temp, pSize);
			cv::resize(temp, temp, cv::Size(temp.cols/2, temp.rows/2));
 
			//cout << "processing image " << argv[i] << endl;
	        //cv_image<bgr_pixel> cimg(temp);
			array2d<rgb_pixel> img;
			dlib::assign_image(img, dlib::cv_image<dlib::bgr_pixel>(temp));
			
			//load_image(img, argv[i]);
			// Make the image larger so we can detect small faces.
			pyramid_up(img);
			std::vector<dlib::rectangle> rects = detector(img);
            std::cout << "Number of faces detected is: " << rects.size() << std::endl;

            //使用shape_predictor来确定每个人脸的bounding boxes在图片中的位置
            std::vector<dlib::full_object_detection> shapes;

			for (unsigned long j = 0; j< rects.size(); j++)
            {
                dlib::full_object_detection shape = sp(img, rects[j]);
                cout << "number of parts: " << shape.num_parts() << endl;
                cout << "pixel position of first part:  " << shape.part(0) << endl;
                cout << "pixel position of second part: " << shape.part(1) << endl;
                shapes.push_back(shape);
			}
			// Now tell the face detector to give us a list of bounding boxes
			// around all the faces in the image.
			// Now we will go ask the shape_predictor to tell us the pose of
			// each face we detected.
			// Now let's view our face poses on the screen.
			//显示识别结果
            window.clear_overlay();
            window.set_image(img);
            window.add_overlay(dlib::render_face_detections(shapes));
        }
    }

    catch (exception& e)
    {
        cout << "\nexception thrown!" << endl;
        cout << e.what() << endl;
    }
}