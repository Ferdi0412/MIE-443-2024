#ifndef BASIC_SOUND_CLIENT_CPP // Include guard
#define BASIC_SOUND_CLIENT_CPP

#include <sound_play/sound_play.h>
#include <ros/console.h>
#include <string>

class SoundPlayer {
    public:
        SoundPlayer();

        playSound( std::string wav_filename );

    private:
        sound_play::SoundClient sc;
        std::string path_to_sounds;
};


SoundPlayer::SoundPlayer( ) {
    path_to_sounds = ros::package::getPath( "mie443_contest3" ) + "/sounds/";
}

SoundPlayer::playSound( std::string wav_filename ) {
    sc.playWave( path_to_sounds + wav_filename );
}


#endif // ~ BASIC_SOUND_CLIENT_CPP
