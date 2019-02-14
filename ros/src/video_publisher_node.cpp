/* video_publisher_node.cpp
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#include <video_publisher/video_publisher_node.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

VideoPublisherNode::VideoPublisherNode(ros::NodeHandle &nh) : nh_(nh), left_it_(nh), right_it_(nh)
{
    left_image_publisher_ = left_it_.advertise("left/image_raw", 1);
    right_image_publisher_ = right_it_.advertise("right/image_raw", 1);

    std::string left_camera_file;
    std::string right_camera_file;    

    if (!nh_.getParam("left_camera_file", left_camera_file) || !nh_.getParam("right_camera_file", right_camera_file))
    {
        ROS_ERROR("Input video file needs to be specified");
    }
    else
    {
        double start_time;
        double stop_time;
        nh_.param<double>("start_time", start_time, 0.0);
        nh_.param<double>("stop_time", stop_time, -1.0);
        publish_stereo_video(left_camera_file, right_camera_file, start_time, stop_time);
    }
}

VideoPublisherNode::~VideoPublisherNode()
{
}

void VideoPublisherNode::publish_stereo_video(const bfs::path &left_video_file_path, const bfs::path &right_video_file_path, double start_time, double stop_time)
{
    cv::VideoCapture left_capture(left_video_file_path.string());
      
    if (!left_capture.isOpened())
    {
        ROS_ERROR("Cound not read %s", left_video_file_path.string().c_str());
        return;
    }

    cv::VideoCapture right_capture(right_video_file_path.string());
      
    if (!right_capture.isOpened())
    {
        ROS_ERROR("Cound not read %s", right_video_file_path.string().c_str());
        return;
    }

    double frames_per_second = left_capture.get(CV_CAP_PROP_FPS);    
    ros::Rate loop_rate(frames_per_second);

    left_capture.set(CV_CAP_PROP_POS_MSEC, start_time);
    right_capture.set(CV_CAP_PROP_POS_MSEC, start_time);

    cv::Mat left_frame;
    cv::Mat right_frame;
    int number_of_frames = 0;

    while(ros::ok())
    {
        left_capture >> left_frame;
        right_capture >> right_frame;
        if (left_frame.empty() || right_frame.empty())
        {
            break;
        }
        number_of_frames++;

        cv_bridge::CvImage left_frame_msg;
        left_frame_msg.encoding = sensor_msgs::image_encodings::BGR8;
        left_frame_msg.image = left_frame;
        left_image_publisher_.publish(left_frame_msg.toImageMsg());

        cv_bridge::CvImage right_frame_msg;
        right_frame_msg.encoding = sensor_msgs::image_encodings::BGR8;
        right_frame_msg.image = right_frame;
        right_image_publisher_.publish(right_frame_msg.toImageMsg());

        if (stop_time > 0.0 && left_capture.get(CV_CAP_PROP_POS_MSEC) >= stop_time)
        {
            break;
        }
        
        loop_rate.sleep();
    }
    ROS_INFO("Number of frames processed: %i", number_of_frames);
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "video_publisher");

    ros::NodeHandle n("~");

    ROS_INFO("[video_publisher] node started");

    VideoPublisherNode video_publisher_node(n); 

    return 0;
}
