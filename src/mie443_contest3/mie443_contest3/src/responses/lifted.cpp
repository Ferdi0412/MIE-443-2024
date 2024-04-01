#ifndef LIFTED_CPP
#define LIFTED_CPP

#include "responses.h"
#include <ros/ros.h>

/**
 * display_discontent - When being lifted off the ground
*/
void display_discontent( SoundPlayer& sound_player, ImageHandler& image_handler ) {
    image_handler.display("confused.png");
    sound_player.play("confused.wav");
    ros::Duration(4);
}


#endif
