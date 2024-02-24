#include <imagePipeline.h>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

//  : matcher_function(&ImagePipeline::match_nothing), draw_nothing(&ImagePipeline::draw_nothing)

ImagePipeline::ImagePipeline(ros::NodeHandle& n, std::string image_topic) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(image_topic, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}



ImagePipeline::ImagePipeline(ros::NodeHandle& n){
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}



void ImagePipeline::setMatcher(bool (*matcher_callback)(cv::Mat, cv::Mat)) {
    matcher_function = matcher_callback;
}



void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }
}



int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/
        // Use: boxes.templates
        cv::imshow("view", img);
        cv::waitKey(10);
    }
    return template_id;
}



int ImagePipeline::getTemplateID_test(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/
        // Use: boxes.templates
        for ( unsigned int i = 0; i < boxes.templates.size(); i++ ) {
            cv::Mat template_img = boxes.templates[i];
            if ( ImagePipeline::matcher_function(img, template_img) ) {
                cv::Mat matched_img = draw_rect_function(img, template_img);
                cv::imshow("match", matched_img);
                cv::waitKey(10);
                return i;
            }
        }
        /***END YOUR CODE***/
        // cv::imshow("view", img);
        // cv::waitKey(10);
    }
    return template_id;
}



bool ImagePipeline::match_nothing( cv::Mat img, cv::Mat template_img ) {
    return false;
}



cv::Mat ImagePipeline::draw_nothing( cv::Mat img, cv::Mat template_img ) {
    return img;
}
