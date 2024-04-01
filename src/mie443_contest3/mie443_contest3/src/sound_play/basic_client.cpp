#ifndef BASIC_SOUND_CLIENT_CPP // Include guard
#define BASIC_SOUND_CLIENT_CPP

#include <sound_play/sound_play.h>
#include <ros/console.h>
#include <string>

class SoundPlayer {
    public:
        void play( std::string wav_filename ) {
            sc.playWave( path_to_sounds + wav_filename );
        }

        SoundPlayer() {
            path_to_sounds = ros::package::getPath( "mie443_contest3" ) + "/sounds/";
        }

    private:
        sound_play::SoundClient sc;
        std::string path_to_sounds;
};

#endif // ~ BASIC_SOUND_CLIENT_CPP
