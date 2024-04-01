/**
 * Use this file to test emotional responses WITHOUT STIMULI...
*/

#include <iostream>

#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>

#include "robot_control/basic_subscriptions.h"
#include "robot_control/basic_publishers.h"
#include "robot_control/program_timer.h"
#include "sound_play/basic_client.cpp"
#include "image_handler/basic_client.cpp"
#include "image_handler/basic_displayer.cpp"
#include "responses/responses.h"

int main ( int argc, char **argv ) {
    // Setup misc. stuff
    ros::init(argc, argv, "response_test");
    ros::NodeHandle nh;

    imageTransporter webcam = subscribe_to_webcam( );

    // Setup sound_play stuff
    SoundPlayer sound_player;
    ImageHandler image_handler(nh);
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
    // sound_player.play("sound.wav");
    // ros::Duration(3).sleep(); // Wait 3 seconds - length of sound.wav file

    // display_discontent(sound_player, image_handler);
    // display_scaredness( sound_player, image_handler );

    while ( within_time_limit(20) ) {
        ros::spinOnce();
        std::cout << "Displaying webcam img...\n";
        display_img( webcam.getImg() );
        if ( webcam.getImg().empty() )
            std::cout << "Empty...\n";

        ros::Duration(1.).sleep();
    }

    // End of MAIN
    return 0;
}
