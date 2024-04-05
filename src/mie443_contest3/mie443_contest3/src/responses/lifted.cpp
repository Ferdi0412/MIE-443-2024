#ifndef LIFTED_CPP
#define LIFTED_CPP

#include "responses.h"
#include <ros/ros.h>

/**
 * display_discontent - When being lifted off the ground
*/
void display_discontent( SoundPlayer& sound_player, ImageHandler& image_handler ) {
    image_handler.display("confused.png");
    sound_player.play("surprised.wav");
    ros::Duration(4).sleep();
}

/**
 * display_scaredness - When lifted too high
*/
void display_scaredness( SoundPlayer& sound_player, ImageHandler& image_handler ) {
    image_handler.display("scared.png");
    sound_player.robot_speech( "I am now scared!" );
    ros::Duration(2).sleep();
}


#endif
