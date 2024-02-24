#pragma once

#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <boxes.h>

class ImagePipeline {
    private:
        cv::Mat img;
        bool isValid;
        image_transport::Subscriber sub;

        /**
         * The following variables can be used to connect to different topics
        */
        const char* kinect_topic = "camera/rgb/image_raw";
        const char* webcam_topic = "camera/image";

    public:
        /**
         * Constructor
         *
         * @param n <ros::NodeHandle> the node handler to set-up the subscription
         * @param image_topic <std::string> the topic to tubscribe to
        */
        ImagePipeline(ros::NodeHandle& n, std::string image_topic);

        /**
         * Constructor -> subscribes to the kinetic image topic
         *
         * @param n <ros::NodeHandle> the node handler to set-up the subscription
        */
        ImagePipeline(ros::NodeHandle& n);

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        int getTemplateID(Boxes& boxes);
};
