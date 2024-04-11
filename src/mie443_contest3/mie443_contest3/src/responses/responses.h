#ifndef RESPONSES_H
#define RESPONSES_H

/**
 * ====================
 * === DEPENDENCIES ===
 * Below are the files that are needed
*/
#include "../robot_control/robot_control.h"

/**
 * frustrated_move_backwards - plays a short sound then moves backwards in hopes of getting past blockage - short wait at back
*/
void frustrated_move_backwards( SoundPlayer& sound_player, ImageHandler& image_handler );

/**
 * rage_move_backwards - plays short sound then moves backwards quicker hoping to get past blockage
*/
void rage_move_backwards( SoundPlayer& sound_player, ImageHandler& image_handler );

/**
 * display_confusion - Don't move, can rotate on spot, indicate confusion
*/
void display_confusion( SoundPlayer& sound_player, ImageHandler& image_handler );

/**
 * display_surprised - When being lifted off the ground
*/
void display_surprised( SoundPlayer& sound_player, ImageHandler& image_handler );

/**
 * display_neutral
*/
void display_neutral( SoundPlayer& sound_player, ImageHandler& image_handler );

/**
 * happy_family
*/
void happy_family( SoundPlayer& sound_player, ImageHandler& image_handler );

/**
 * ===============
 * === PACKAGE ===
 * Below are the files that are included
 * NOTE: These MUST be the very last thing in this file, and must all include inclusion guards
*/
#include "blocked_path.cpp"
#include "lost_person.cpp"
#include "lifted.cpp"
#include "neutral.cpp"
#include "family.cpp"

#endif // ~ RESPONSES_H
