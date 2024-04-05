/**
 * When path is blocked, can be FRUSTRATED or ANGRY
*/
#include "responses.h"

/**
 * frustrated_move_backwards - plays a short sound then moves backwards in hopes of getting past blockage - short wait at back
*/


void happy_family( SoundPlayer& sound_player, ImageHandler& image_handler ) {
    long long start_time = seconds_elapsed();
    image_handler.display("family.png");
    sound_player.play("family.wav");
    ros::Duration(10).sleep();
    while ( (seconds_elapsed() - start_time < 10) /*&& face_detector.isFaceDetected()*/ ){
        ros::Duration(0.5).sleep();
    }

}

