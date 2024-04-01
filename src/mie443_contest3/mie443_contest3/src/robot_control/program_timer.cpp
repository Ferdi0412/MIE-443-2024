#ifndef PROGRAM_TIMER_CPP
#define PROGRAM_TIMER_CPP

#include "program_timer.h"
#include <chrono>

typedef std::chrono::time_point<std::chrono::system_clock> _sys_time_t;

static _sys_time_t start_time;
static bool        is_started = false;

inline _sys_time_t get_time() {
    return std::chrono::system_clock::now();
}

inline long seconds_between( _sys_time_t begin_time, _sys_time_t end_time ) {
    return std::chrono::duration_cast<std::chrono::seconds>( end_time - begin_time ).count();
}

long seconds_elapsed( ) {
    if ( !is_started ) {
        start_time = get_time();
        is_started = true;
    }

    return seconds_between( start_time, get_time() );
}

bool within_time_limit( long time_limit ) {
    return seconds_elapsed() <= time_limit;
}

#endif // ~ PROGRAM_TIMER_CPP
