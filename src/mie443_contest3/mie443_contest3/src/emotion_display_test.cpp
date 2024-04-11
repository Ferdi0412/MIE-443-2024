#include <iostream>
#include <string>

#include <header.h>

#include "robot_control/robot_control.h"
#include "image_handler/basic_client.cpp"
#include "sound_play/basic_client.cpp"
#include "responses/responses.h"

int main ( int argc, char ** argv ) {
    ros::init( argc, argv, "emotion_test" );
    ros::NodeHandle nh;
    ros::Rate loop_rate(0.5);

    SoundPlayer  sound_player;
    ImageHandler image_handler(nh);

    initialize_all( nh );

    while ( 1 ) {
        std::cout << "PLEASE INPUT A NUMBER FROM 0 TO 7 OR h FOR HELP:\n";
        std::string user_input;
        if ( std::cin.eof() )
            break;
        std::cin >> user_input;

        if ( !user_input.empty() ) {
            switch( user_input[0] ) {
                case 'h':
                    std::cout << "Options:\n0: neutral\n1: confusion\n2: saddness\n3: happy\n4: scared\n 5: discontent!\n6: frustrated\n7: rage\n";
                    break;

                case '0':
                    std::cout << "Neutral!\n";
                    display_neutral( sound_player, image_handler );
                    break;

                case '1':
                    std::cout << "Confusion!\n";
                    display_confusion( sound_player, image_handler );
                    break;

                case '2':
                    std::cout << "Sadness!\n";
                    display_sadness( sound_player, image_handler );
                    break;

                case '3':
                    std::cout << "Family!\n";
                    happy_family( sound_player, image_handler );
                    break;

                case '4':
                    std::cout << "Scared!\n";
                    display_scaredness( sound_player, image_handler );
                    break;

                case '5':
                    std::cout << "Discontent!\n";
                    display_discontent( sound_player, image_handler );
                    break;

                case '6':
                    std::cout << "bumped few\n";
                    frustrated_move_backwards( sound_player, image_handler );
                    break;

				case '7':
                    std::cout << "Bumped MANY\n";
					rage_move_backwards( sound_player, image_handler );
                    break;

                default:
                    std::cout << "Could not recognize input! Please try again...\n";
                    break;
            }
        }

        loop_rate.sleep();
    }
}
