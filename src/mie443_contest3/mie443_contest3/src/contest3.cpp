#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>

// #include "kinect_face_detector.hpp" // added for face detection
#include "robot_control/robot_control.h"
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

	initialize_robot_subscriptions(    nh );
	initialize_follower_subscriptions( nh );
	initialize_basic_movers(           nh );
	initialize_move_robot(             nh );

	// Setup state trackers...
	Stimuli robot_state = FOLLOWING;
	Stimuli prev_state  = FOLLOWING;
	uint8_t bump_count  = 0;
	long long lost_time = 0;
	bool lost_once      = false;

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

		/**
		 * =================
		 * === GET STATE ===
		 * =================
		*/
		/* If robot was lifted off the ground... */
		if ( check_raised() )
			robot_state = LIFTED;

		/* If essentially stood still, count number of faces to trigger happy */
		else if ( (fabs(get_odom_lin_velocity()) < 0.05) && (fabs(get_odom_rot_velocity()) < 0.1) && (numberOfFaces() >= 3) )
		 	robot_state = FAMILY_DETECTED;

		/* Target was lost... */
		else if ( !get_target_available() )
			robot_state = PERSON_LOST;

		/* If bumpers triggered... */
		else if ( check_bumpers() )
			robot_state = PATH_BLOCKED;

		/* Otherwise follow the target... */
		else // get_target_available == True
			robot_state = FOLLOWING;



		/**
		 * ====================
		 * === ACT ON STATE ===
		 * ====================
		*/
		/* Finite State Machine section - display emotions and/or detect higher level stimuli (if applicable) */
		switch ( robot_state ) {
			/* 1. FOLLOWING */
			case FOLLOWING:
				if ( prev_state != FOLLOWING )
					display_neutral( sound_player, image_handler );

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
				if ( prev_state != PERSON_LOST ) {
					lost_once = false;
					lost_time = seconds_elapsed();
				}

				if ( lost_once ) {
					display_sadness(sound_player, image_handler);
				}

				else if ( (seconds_elapsed() - lost_time) > 3 ) {
					display_confusion(sound_player, image_handler);
					lost_once = true;
				}
				// Confused - look around - start timer?
				// If long time yet not found - sad
				break;


			/* 4. FAMILY_DETECTED */
			case FAMILY_DETECTED:
				/* Fill in FAMILY_DETECTED here... */
				// Stop and make a happy remark or something before moving towards it???
				happy_family(sound_player, image_handler);
				break;


			/* 5. LIFTED */
			case LIFTED:
				/* Fill in LIFTED here... */
				if ( prev_state != LIFTED )
					display_surprised( sound_player, image_handler );
				break;

			// There should be no reason to add default....
			// default:
		}

		loop_rate.sleep();
	}

	return 0;
}
