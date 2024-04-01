#ifndef LOST_PERSON_CPP
#define LOST_PERSON_CPP

/**
 * When it looses track of the person, gets CONFUSED then SAD
*/

#include "responses.h"

#include <ros/ros.h>

/**
 * display_confusion - Don't move, can rotate on spot, indicate confusion
*/
void display_confusion( SoundPlayer& sound_player, ImageHandler& image_handler ) {
    image_handler.display("confused.png");
    sound_player.play("confused.wav");
    ros::Duration(2.5).sleep();

    // Add a slow rotation here - look side to side
}

/**
 * display_sadness - Don't move, can rotate on spot, indicate sadness
*/
void display_sadness( SoundPlayer& sound_player, ImageHandler& image_handler ) {
    image_handler.display("sad.jpg");
    // sound_player.play("");
    ros::Duration(2.5).sleep();
}

#endif // ~ LOST_PERSON_CPP
