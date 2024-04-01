#ifndef BASIC_IMAGE_CLIENT_CPP
#define BASIC_IMAGE_CLIENT_CPP

#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

class ImageHandler {
    public:
        void display( std::string jpg_png_filename ) {
            std_msgs::String msg;
            msg.data = path_to_files + "/images/" + jpg_png_filename;
            img_pub.publish( msg );
        }

        void play ( std::string mp4_filename ) {

        }

        ImageHandler( ros::NodeHandle node_handler ) {
            path_to_files = ros::package::getPath( "mie443_contest3" );
            img_pub       = node_handler.advertise<std_msgs::String>( "/emotional_display/image", 1 );
        }

    private:
        std::string path_to_files;
        ros::Publisher img_pub;

};


#endif // ~ BASIC_IMAGE_CLIENT_CPP
