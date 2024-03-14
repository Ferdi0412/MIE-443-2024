#include <imagePipeline.h>
#include "parin-functions.cpp"

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" //Kinect:"camera/rgb/image_raw" webcam:"camera/image"



/**
 * === CONSTRUCTORS ===
*/
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

/**
 * === PRIVATE METHODS ===
*/
bool ImagePipeline::match_nothing( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img ) {
    return false;
}

cv::Mat ImagePipeline::draw_nothing( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img ) {
    return img;
}

int ImagePipeline::match_no_boxes( const cv::Mat& img, const std::vector<cv::Mat>& template_imgs ) {
    return -1;
}

/**
 * === CALLBAK SETTERS ===
*/
void ImagePipeline::setMatchFunction( int (*match_callback)(const cv::Mat&, const std::vector<cv::Mat>&)) {
    match_function = match_callback;
}

/**
 * === SUBSCRIPTION CALLBACK ===
*/
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


/**
 * === SEARCH FUNCTIONS ===
*/
int ImagePipeline::getTemplateID( const Boxes& boxes ) {
    int template_id = -1;

    // Class error handling
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    }
    // Empty image handling
    else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    }
    // Image search
    else {
        template_id = match_function(img, boxes.templates); // should search video feed for the matching image tags from the vector
        std::cout << "Template ID: " << template_id << std::endl; 
        cv::imshow("Kinect image", img);
        cv::waitKey(10);
    }
    return template_id;
}
