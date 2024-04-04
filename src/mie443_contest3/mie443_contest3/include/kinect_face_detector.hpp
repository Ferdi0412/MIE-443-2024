#ifndef KINECT_FACE_DETECTOR_HPP
#define KINECT_FACE_DETECTOR_HPP

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/utils/filesystem.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

class KinectFaceDetector {
public:
    KinectFaceDetector();

    bool isFaceDetected();

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    cv::CascadeClassifier face_cascade;
    bool face_detected_; // New member variable
};

#endif /* KINECT_FACE_DETECTOR_HPP */
