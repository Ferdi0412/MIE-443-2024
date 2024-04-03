#include "kinect_face_detector.hpp"

KinectFaceDetector::KinectFaceDetector() : face_detected_(false) {
    if (!face_cascade.load(cv::samples::findFile("haarcascade_frontalface_alt.xml"))) {
        ROS_ERROR("Failed to load face cascade");
        return;
    }

    image_sub_ = nh_.subscribe("/camera/rgb/image_raw", 1, &KinectFaceDetector::imageCallback, this);
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

        cv::imshow("Face detection", cv_ptr->image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

bool KinectFaceDetector::isFaceDetected() const {
    return face_detected_;
}
