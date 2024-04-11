#ifndef LIFTED_CPP
#define LIFTED_CPP

#include "responses.h"
#include <ros/ros.h>

/**
 * display_surprised - When being lifted off the ground
*/
void display_surprised( SoundPlayer& sound_player, ImageHandler& image_handler ) {
    image_handler.display("surprised.png");
    sound_player.play("surprised.wav");
    ros::Duration(0.4);
}



#endif
