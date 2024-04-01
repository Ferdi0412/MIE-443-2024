/**
 * Use this file to test emotional responses WITHOUT STIMULI...
*/

#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>

#include "robot_control/basic_subscriptions.h"
#include "sound_play/basic_client.cpp"

int main ( int argc, char **argv ) {
    // Setup misc. stuff
    ros::init(argc, argv, "response_test");
    ros::NodeHandle nh;

    // Setup sound_play stuff
    SoundPlayer sound_player;
    // sound_play::SoundClient sc;
    // std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";

    /* Add any more setup stuff here... */

    // Wait to (hopefully) ensure that soundclient subscription is working...
    ros::Duration(1).sleep();

    /**
     * ============
     * === MAIN ===
    */

    /* Add test stuff here... */
    // eg., play some sound:
    sound_player.play("sound.wav");
    ros::Duration(3).sleep(); // Wait 3 seconds - length of sound.wav file

    // End of MAIN
    return 0;
}
