// https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29
// https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient

#include <string>
#include <list>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
 * ==============
 * === PARAMS ===
*/
#define WAITKEY_TIME 25

/**
 * =========================
 * === KEEP LATEST IMAGE ===
*/
static cv::Mat latest_img;

void display_latest_img( std::string window_name = "EmotionalDisplay" ) {
    if ( latest_img.empty() )
        return;
    cv::imshow( window_name, latest_img );
    cv::waitKey( WAITKEY_TIME );
}

/**
 * ===============================
 * === DISPLAYING STATIC IMAGE ===
*/
ros::Subscriber img_sub;
std::string     next_img;

void img_callback( const std_msgs::String::ConstPtr& msg ) {
    next_img = msg->data;
}

void show_img( std::string file_path, std::string window_name = "EmotionalDisplay" ) {
    cv::Mat img = cv::imread( file_path, cv::IMREAD_COLOR );
    if ( img.empty() )
        return;
    latest_img = img;
    cv::imshow( window_name, img );
    cv::waitKey( WAITKEY_TIME );
}

/**
 * =========================
 * === DISPLAYING VIDEOS ===
*/
ros::Subscriber video_sub;
std::string     next_video;

void display_vid( std::string file_path, std::string window_name = "EmotionalDisplay" ) {
    
    next_video = "";
}


/**
 * ============
 * === MAIN ===
*/
int main( int argc, char **argv ) {
    ros::init( argc, argv, "image_displayer" );
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    img_sub = nh.subscribe( "/emotional_display/image", 1, &img_callback );

    while ( ros::ok() ) {
        ros::spinOnce();

        /* Data here... */
        if ( !next_video.empty() ) {
            next_img = "";
            display_vid( next_video );
        }

        else if ( !next_img.empty() ) {
            next_video = "";
            show_img( next_img );
            next_img = "";
        } 

        else {
            display_latest_img();
        }

        loop_rate.sleep();
    }
}