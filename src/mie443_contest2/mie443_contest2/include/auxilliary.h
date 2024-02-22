/**
 * Here are some auxilliary functions intended to make building the main program easier.
*/
#ifndef AUXILLIARY_H
#define AUXILLIARY_H

#include <chrono>



/**
 * =============
 * === TIMER ===
 *
 * The following functions should make controlling the time the program runs for...
*/
typedef std::chrono::time_point<std::chrono::system_clock> aux_timer_t;

/**
 * setMainStartTime must run once at the start of the program, for the other TIME functions to work
*/
void mainTimerStart( void );

/**
 * getMainSecondsElapsed will return the number of seconds since setMainStartTime was run
 *
 * @returns <uint64_t> the number of seconds
*/
uint64_t mainTimerSecondsElapsed( void );

/**
 * getTimer will return a timer
 *
 * @returns <aux_timer_t> a timer
*/
aux_timer_t getTimer( void );

/**
 * getSecondsElapsed will return the number of seconds since getTimer was run
 *
 * @returns <uint64_t> the number of seconds
*/
uint64_t getSecondsElapsed( aux_timer_t input_timer );

/**
 * getMillisecondsElapsed will return the number of milliseconds since getTimer was run
 *
 * @returns <uint64_t> the number of milliseconds
*/
uint64_t getMillisecondsElapsed( aux_timer_t input_timer );



#endif
