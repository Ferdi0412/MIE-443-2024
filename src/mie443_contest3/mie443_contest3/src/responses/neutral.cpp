#ifndef NEUTRAL_CPP
#define NEUTRAL_CPP

#include "responses.h"

void display_neutral( SoundPlayer& sound_player, ImageHandler& image_handler ) {
    image_handler.display("neutral.png"); // Add some "neutral" image
    sound_player.stop();
}



#endif
