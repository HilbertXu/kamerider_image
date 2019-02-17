/*
Date: 2019/02/02
Author: Xu Yucheng
Abstract: use openpose in ros
*/
// Command-line user intraface
#define OPENPOSE_FLAGS_DISABLE_PRODUCER
#define OPENPOSE_FLAGS_DISABLE_DISPLAY
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>

// ros headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// opencv headers
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// std headers
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <string>

using namespace std;
using namespace cv;
using namespace op;

class openpose_ros
{
private:
    std::string sub_image_raw_topic_name;
    ros::Subscriber sub_image;

    void configureWrapper(op::Wrapper& opWrapper)
    {
        try
        {
            // Configuring OpenPose

            // logging_level
            op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
                    __LINE__, __FUNCTION__, __FILE__);
            op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
            op::Profiler::setDefaultX(FLAGS_profile_speed);

            // Applying user defined configuration - GFlags to program variables
            // outputSize
            const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
            // netInputSize
            const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
            // faceNetInputSize
            const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
            // handNetInputSize
            const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
            // poseMode
            const auto poseMode = op::flagsToPoseMode(FLAGS_body);
            // poseModel
            const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
            // JSON saving
            if (!FLAGS_write_keypoint.empty())
                op::log("Flag `write_keypoint` is deprecated and will eventually be removed."
                        " Please, use `write_json` instead.", op::Priority::Max);
            // keypointScaleMode
            const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
            // heatmaps to add
            const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                        FLAGS_heatmaps_add_PAFs);
            const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
            // >1 camera view?
            const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
            // Face and hand detectors
            const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
            const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
            // Enabling Google Logging
            const bool enableGoogleLogging = true;

            // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
            const op::WrapperStructPose wrapperStructPose{
                poseMode, netInputSize, outputSize, keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start,
                FLAGS_scale_number, (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
                poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
                FLAGS_part_to_show, FLAGS_model_folder, heatMapTypes, heatMapScaleMode, FLAGS_part_candidates,
                (float)FLAGS_render_threshold, FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max,
                FLAGS_prototxt_path, FLAGS_caffemodel_path, (float)FLAGS_upsampling_ratio, enableGoogleLogging};
            opWrapper.configure(wrapperStructPose);
            // Face configuration (use op::WrapperStructFace{} to disable it)
            const op::WrapperStructFace wrapperStructFace{
                FLAGS_face, faceDetector, faceNetInputSize,
                op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
                (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold};
            opWrapper.configure(wrapperStructFace);
            // Hand configuration (use op::WrapperStructHand{} to disable it)
            const op::WrapperStructHand wrapperStructHand{
                FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
                op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
                (float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold};
            opWrapper.configure(wrapperStructHand);
            // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
            const op::WrapperStructExtra wrapperStructExtra{
                FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads};
            opWrapper.configure(wrapperStructExtra);
            // Output (comment or use default argument to disable any output)
            const op::WrapperStructOutput wrapperStructOutput{
                FLAGS_cli_verbose, FLAGS_write_keypoint, op::stringToDataFormat(FLAGS_write_keypoint_format),
                FLAGS_write_json, FLAGS_write_coco_json, FLAGS_write_coco_foot_json, FLAGS_write_coco_json_variant,
                FLAGS_write_images, FLAGS_write_images_format, FLAGS_write_video, FLAGS_write_video_fps,
                FLAGS_write_video_with_audio, FLAGS_write_heatmaps, FLAGS_write_heatmaps_format, FLAGS_write_video_3d,
                FLAGS_write_video_adam, FLAGS_write_bvh, FLAGS_udp_host, FLAGS_udp_port};
            opWrapper.configure(wrapperStructOutput);
            // No GUI. Equivalent to: opWrapper.configure(op::WrapperStructGui{});
            // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
            if (FLAGS_disable_multi_thread)
                opWrapper.disableMultiThreading();
        }
        catch (const std::exception& e)
        {
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }


    void imageCallback (const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;
    }

    int tutorialApiCpp()
    {
        try
        {
            op::log("Starting OpenPose demo...", op::Priority::High);
            const auto opTimer = op::getTimerInit();

            // Configuring OpenPose
            op::log("Configuring OpenPose...", op::Priority::High);
            op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
            configureWrapper(opWrapper);

            // Starting OpenPose
            op::log("Starting thread(s)...", op::Priority::High);
            opWrapper.start();

            // Read frames on directory
            const auto imagePaths = op::getFilesOnDirectory(FLAGS_image_dir, op::Extensions::Images);

            // Process and display images
            for (const auto& imagePath : imagePaths)
            {
                const auto imageToProcess = cv::imread(imagePath);
                auto datumProcessed = opWrapper.emplaceAndPop(imageToProcess);
                if (datumProcessed != nullptr)
                {
                    printKeypoints(datumProcessed);
                    if (!FLAGS_no_display)
                    {
                        const auto userWantsToExit = display(datumProcessed);
                        if (userWantsToExit)
                        {
                            op::log("User pressed Esc to exit demo.", op::Priority::High);
                            break;
                        }
                    }
                }
                else
                    op::log("Image " + imagePath + " could not be processed.", op::Priority::High);
            }

            // Measuring total time
            op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);

            // Return
            return 0;
        }
        catch (const std::exception& e)
        {
            return -1;
        }
    }
};

