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
         * The following variable will be used as a variable to return if an image is a match of a template
        */
        bool (*search_function)(const cv::Mat&, unsigned int, const cv::Mat&) = &match_nothing;
        cv::Mat (*draw_function)(const cv::Mat&, unsigned int, const cv::Mat&) = &draw_nothing;

    public:
        /**
         * The following variables can be used to connect to different topics
        */
        const char* kinect_topic = "camera/rgb/image_raw";
        const char* webcam_topic = "camera/image";

    private:
        static bool match_nothing( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img );

        static cv::Mat draw_nothing( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img );

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

        /**
         * setTemplateSearcher will set a callback method that takes the image and a template to return a bool on whether it's a match or not
         *
         * @param search_callback(img, template_no, template_img) a callback that takes the most recent image and a template and returns true if the template is detected in the image
        */
        void setTemplateSearcher(bool (*search_callback)(const cv::Mat&, unsigned int, const cv::Mat&));

        /**
         * setImageDrawer will set a callback that takes an image and a template and update it to visualize matches
         *
         * @param draw_callback(img, template_no, template_img) a callback that takes the most recent image and a template and adds a rectangle to outline the match
        */
        void setImageDrawer(cv::Mat (*draw_callback)(const cv::Mat&, unsigned int, const cv::Mat&));

        /**
         * imageCallback runs to update the subscription
        */
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        /**
         * getTemplateID is the default provided image match function
         *
         * @param boxes the boxes class that defines the templates to search for
        */
        int getTemplateID(Boxes& boxes);
};
