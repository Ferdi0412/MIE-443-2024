#ifndef LIFTED_CPP
#define LIFTED_CPP

#include "responses.h"
#include <ros/ros.h>

/**
 * display_surprised - When being lifted off the ground
*/
void display_surprised( SoundPlayer& sound_player, ImageHandler& image_handler ) {
    image_handler.display("surprised.jpg");
    sound_player.play("surprised2.wav");
    ros::Duration(0.8);
}



#endif
