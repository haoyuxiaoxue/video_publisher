/* video_publisher_node.h
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#ifndef VIDEO_PUBLISHER_NODE_H_
#define VIDEO_PUBLISHER_NODE_H_

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <image_transport/image_transport.h>

namespace bfs = boost::filesystem;

/**
 * This class publishes frames from an input video file to a sensor_msgs/Image topic at the frame rate of the video
 * The video is read using OpenCV and converted to the ros Image message using cv_bridge
 */
class VideoPublisherNode
{
    public:
        /**
         * Constructor
         *
         * @param nh
         * ROS NodeHandle object
         */
        VideoPublisherNode(ros::NodeHandle &nh);

        /**
         * Destructor
         */
        virtual ~VideoPublisherNode();


    private:
        /**
         * Reads video file at video_file_path and publishes frames to output image topic
         *
         * @param video_file_path
         * Full path to input video file
         *
         * @param start_time
         * Time in video to start publishing frames from (in milliseconds)
         *
         * @param stop_time
         * Time in video to stop publishing frames (in milliseconds)
         * Set to negative if frames till the end of the file are to be published
         */
        void publish_stereo_video(const bfs::path &left_video_file_path, const bfs::path &right_video_file_path, double start_time, double stop_time);

    private:
        /**
         * ROS NodeHandle object
         */
        ros::NodeHandle nh_;

        /**
         * ImageTransport object
         */
        image_transport::ImageTransport left_it_;
        image_transport::ImageTransport right_it_;
        
        /**
         * Image publisher for frames from the video
         */
        image_transport::Publisher left_image_publisher_;
        image_transport::Publisher right_image_publisher_;
};

#endif
