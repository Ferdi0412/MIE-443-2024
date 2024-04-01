#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>

#include "robot_control/basic_subscriptions.h"
#include "robot_control/basic_publishers.h"
#include "robot_control/program_timer.h"
#include "sound_play/basic_client.cpp"
#include "image_handler/basic_client.cpp"
// #include "image_handler/basic_displayer.cpp"
#include "responses/responses.h"

using namespace std;

enum Stimuli {
	FOLLOWING,
	PERSON_LOST,
	PERSON_FAR,
	PATH_BLOCKED,
	LIFTED,
	FAMILY_DETECTED
};

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	/**
	 * =============
	 * === SETUP ===
	 * =============
	*/
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;
	ros::Rate loop_rate(10); // 10 Hz - max. frequency of main while loop

	// Camera subscribers
	imageTransporter rgbTransport   = subscribe_to_webcam(); // subscribe_to_kinect();
	imageTransporter depthTransport = subscribe_to_depth_sensor();

	// Custom setup
	SoundPlayer sound_player;
	ImageHandler image_handler(nh);
	ros::Duration(0.5).sleep(); // Give the sound_player time to connect to the sound_play node

	initialize_robot_subscriptions( nh );
	initialize_follower_subscriptions( nh );
	initialize_basic_movers( nh );

	// Setup state trackers...
	Stimuli robot_state = FOLLOWING;
	Stimuli prev_state  = FOLLOWING;
	uint8_t bump_count  = 0;

	if ( !wait_for_odom_msg(nh, 2.) ) {
		ROS_ERROR("Did not receive odometry message before wait_for_odom_msg timeout...\n");
		exit(-1);
	}
	double ground_z = get_odom_z();



	// ----------------------------------------------------------
	/**
	 * =================
	 * === MAIN LOOP ===
	 * =================
	*/
	while( ros::ok() && within_time_limit(480) ){
		ros::spinOnce();
		prev_state = robot_state; // To for example trigger reaction on first occurance - eg. play surprise sound once when lifted...

		/* In order to set the state correctly, the order of the checks must be correct.
		1. Check if lifted    - LIFTED
		2. Check for faces    - FAMILY_DETECTED
		3. Check if no target - PERSON_LOST/PERSON_FAR
		4. Check if bumper    - PATH_BLOCKED
		5. Check if normal    - FOLLOWING
		*/
		if ( check_raised() )
			robot_state = LIFTED;
		// else if ( ... Check For Faces )
		// 	robot_state = FAMILY_DETECTED;
		else if ( !get_target_available() )
			robot_state = PERSON_LOST;
		else if ( check_bumpers() )
			robot_state = PATH_BLOCKED;
		else // get_target_available == True
			robot_state = FOLLOWING;

		/* Finite State Machine section - display emotions and/or detect higher level stimuli (if applicable) */
		switch ( robot_state ) {
			/* 1. FOLLOWING */
			case FOLLOWING:
				if ( prev_state != FOLLOWING )
					display_neutral( sound_player, image_handler );
				/* Fill in FOLLOWING here... */
				// eg.
				publish_velocity( get_follower_cmd() ); // Follow the follower node's path to person
				break;


			/* 2. PATH_BLOCKED */
			case PATH_BLOCKED:
				/* Fill in PATH_BLOCKED here... */
				bump_count ++;
				if ( bump_count < 3 )
					frustrated_move_backwards( sound_player, image_handler );
				else
					rage_move_backwards( sound_player, image_handler ); // Rage
				// Have a move_backwards function...
				break;


			/* 3. PERSON_LOST */
			case PERSON_LOST:
				/* Fill in PERSON_LOST here... */
				// Confused - look around - start timer?
				// If long time yet not found - sad
				break;


			/* 4. FAMILY_DETECTED */
			case FAMILY_DETECTED:
				/* Fill in FAMILY_DETECTED here... */
				// Stop and make a happy remark or something before moving towards it???
				break;


			/* 5. LIFTED */
			case LIFTED:
				/* Fill in LIFTED here... */
				if ( (get_odom_z() - ground_z) > 0.2 )
					display_scaredness( sound_player, image_handler );

				else if ( prev_state != LIFTED )
					display_discontent( sound_player, image_handler );

				break;

			// There should be no reason to add default....
			// default:
		}

		loop_rate.sleep();
	}

	return 0;
}
