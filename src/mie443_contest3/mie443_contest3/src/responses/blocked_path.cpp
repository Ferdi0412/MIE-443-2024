/**
 * When path is blocked, can be FRUSTRATED or ANGRY
*/
#include "responses.h"
#include "move_robot.h"

/**
 * frustrated_move_backwards - plays a short sound then moves backwards in hopes of getting past blockage - short wait at back
*/
void frustrated_move_backwards( SoundPlayer& sound_player, ImageHandler& image_handler ) {
    image_handler.display("angry.png");
    sound_player.play("angry.wav");
    move_forwards(-2);
    // *** Have a turn 45deg function to move back and turn and then move forward ***
    ros::Duration(2).sleep();
}

/**
 * rage_move_backwards - plays short sound then moves backwards quicker hoping to get past blockage
*/
void rage_move_backwards( SoundPlayer& sound_player, ImageHandler& image_handler ) {
    
}
