#ifndef PROGRAM_TIMER_H
#define PROGRAM_TIMER_H



/**
 * seconds_elapsed - returns the number of seconds since the start of the program
 *
 * The start time is set upon the first call to seconds_elapsed
*/
long seconds_elapsed();

/**
 * nanoseconds_elapsed - same as seconds_elapsed but using nanoseconds
*/
long nanoseconds_elapsed();

/**
 * within_time_limit - returns true if the program is within the time limit
 *
 * @param time_limit <long> Maximum number of seconds
*/
bool within_time_limit( long time_limit );



/**
 * ======================
 * === IMPLEMENTATION ===
 * ======================
*/

#include "src/program_timer.cpp"

#endif // ~ PROGRAM_TIMER_H
