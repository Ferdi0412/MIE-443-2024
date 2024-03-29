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
        int (*match_function)( const cv::Mat&, const std::vector<cv::Mat>& ) = &match_no_boxes;


    public:
        /**
         * The following variables can be used to connect to different topics
        */
        const char* kinect_topic = "camera/rgb/image_raw";
        const char* webcam_topic = "camera/image";

    private:
        static bool match_nothing( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img );

        static cv::Mat draw_nothing( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img );

        static int match_no_boxes( const cv::Mat& img, const std::vector<cv::Mat>& template_imgs);

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
         * setMatchFunction will set a callback that takes an image and the boxes object to match the input img
         *
         * @param match_callback(img, boxes) a callback that takes the most recent image to match to a template in boxes
        */
        void setMatchFunction(int (*match_callback)(const cv::Mat&, const std::vector<cv::Mat>&));

        /**
         * imageCallback runs to update the subscription
        */
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        /**
         * getTemplateID is the default provided image match function
         *
         * @param boxes the boxes class that defines the templates to search for
         * @implements match_function as set using setMatchCallback
         *
         * @returns -1 on fail, otherwise the index/id of the box matched from boxes
        */
        int getTemplateID(const Boxes& boxes);

        /**
         * getKinectImage returns the kinect image
         *
         * @note no safety stuff...
        */
        cv::Mat getKinectImage( void );
};
