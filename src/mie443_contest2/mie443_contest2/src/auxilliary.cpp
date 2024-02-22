#include <auxilliary.h>

/**
 * =============
 * === TIMER ===
*/
static aux_timer_t main_clock;

void mainTimerStart( void ) {
    main_clock = getTimer();
}

uint64_t mainTimerSecondsElapsed( void ) {
    return getSecondsElapsed( main_clock );
}

aux_timer_t getTimer( void ) {
    return std::chrono::system_clock::now();
}

uint64_t getSecondsElapsed( aux_timer_t input_timer ) {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - input_timer).count();
}

uint64_t getMillisecondsElapsed( aux_timer_t input_timer ) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - input_timer).count();
}
