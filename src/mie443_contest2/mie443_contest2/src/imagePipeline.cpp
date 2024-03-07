#include <imagePipeline.h>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

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
void ImagePipeline::setTemplateSearcher(bool (*search_callback)(const cv::Mat&, unsigned int, const cv::Mat&)) {
    search_function = search_callback;
}

void ImagePipeline::setImageDrawer( cv::Mat (*draw_callback)(const cv::Mat&, unsigned int, const cv::Mat&)) {
    draw_function = draw_callback;
}

void ImagePipeline::setBoxMatcher( int (*match_callback)(const cv::Mat&, const std::vector<cv::Mat>&)) {
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
int ImagePipeline::getTemplateID(Boxes& boxes) {
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
        /***YOUR CODE HERE***/
        // Use: boxes.templates
        for ( unsigned int i = 0; i < boxes.templates.size(); i++ ) {
            cv::Mat template_img = boxes.templates[i];
            if ( ImagePipeline::search_function(img, i, template_img) ) {
                cv::Mat matched_img = draw_function(img, i, template_img);
                cv::imshow("match", matched_img);
                cv::waitKey(10);
                // Exit on first match...
                return i;
            }
        }
        /***END YOUR CODE***/

        // Commented out to only show image upon a match...
        // cv::imshow("view", img);
        // cv::waitKey(10);
    }
    return template_id;
}

int ImagePipeline::getTemplateID_v2( const Boxes& boxes ) {
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
        template_id = match_function(img, boxes.templates);
    }
    return template_id;
}
