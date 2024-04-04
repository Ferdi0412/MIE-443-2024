#include "kinect_face_detector.hpp"
#include <opencv2/highgui.hpp>

KinectFaceDetector::KinectFaceDetector() : face_detected_(false) {
    // Specify the path to the XML file
    std::string cascade_path = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";

    // Load the cascade classifier
    if (!face_cascade.load(cascade_path)) {
        ROS_ERROR("Failed to load face cascade");
        return;
    }

    // Subscribe to the image topic
    image_sub_ = nh_.subscribe("/camera/rgb/image_raw", 1, &KinectFaceDetector::imageCallback, this);

    // Create a window to display the live feed
    cv::namedWindow("Face Detection", cv::WINDOW_NORMAL);
}

void KinectFaceDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        cv::Mat frame_gray;
        cv::cvtColor(cv_ptr->image, frame_gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(frame_gray, frame_gray);

        std::vector<cv::Rect> faces;
        face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

        face_detected_ = !faces.empty(); // Update the face_detected_ flag

        for(size_t i = 0; i < faces.size(); i++) {
            cv::Point center(faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2);
            cv::ellipse(cv_ptr->image, center, cv::Size(faces[i].width/2, faces[i].height/2), 0, 0, 360, cv::Scalar(255, 0, 255), 4);
        }

        // Display the live feed with face detection
        cv::imshow("Face Detection", cv_ptr->image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

bool KinectFaceDetector::isFaceDetected() {
    return face_detected_;
}
