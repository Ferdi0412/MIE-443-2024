#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define WAITKEY_TIME 25

/**
 * Display an image
*/
void display_img( cv::Mat img, std::string window_name = "InlineDisplay" ) {
    if ( img.empty() )
        return;
    cv::imshow( window_name, img );
    cv::waitKey( WAITKEY_TIME );
}
