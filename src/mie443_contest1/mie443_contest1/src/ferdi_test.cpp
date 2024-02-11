// STD imports
#include <vector>
#include <cmath>
#include <chrono>
#include <stdint.h>
#include <stdio.h>

// ROS imports
#include <ros/console.h>
#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

// Team1::Robot import
#include "robot.cpp"

// LinearAlgebra functions
#include "include/lin_approx.hpp"
#include "include/lin_approx.cpp"

// Temp imports
#include <cmath>
#include <iostream>
#include <vector>


/**
 * ==============================
 * === FUNCTIONS DECLARATIONS ===
 * ==============================
*/

/**
 * secondsElapsed
 *
 * @returns number of seconds from program_start
*/
uint16_t secondsElapsed(void);

/**
 * printVectorFloats prints each float in a vector
*/
void printVectorFloats( const std::vector<float>& the_vector );

/**
 * =====================
 * === GLOBAL params ===
 * =====================
*/
#define R_SQUARED_THRESHOLD 0.7
static std::chrono::time_point<std::chrono::system_clock> program_start;

static const unsigned long long program_duration = 10;

#define SPEED_HIGH 0.2
// #define ROT_HIGH 0.2

float arr[] = { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};


/**
 * ============
 * === MAIN ===
 * ============
 *
 * @param argc number of params used in starting program (ignore but keep)
 * @param argv string params used when starting program  (ignore but keep)
*/
int main ( int argc, char **argv ) {
    std::vector<float> temp_v;
    temp_v.assign( arr, arr + sizeof(arr) / sizeof(arr[0]) );

    std::cout << temp_v.size() << std::endl;
    lin_approx_t temp = linearApproximation( temp_v, 0, temp_v.size() );
    std::cout << "Is there an error? " << checkApproximationError(temp) << std::endl;
    std::cout << (isStraightLine( temp, R_SQUARED_THRESHOLD ) ? "IT IS STRAIGHT ENOUGH!" : "It isn't straight enought :(") << std::endl;
    std::cout << "R-Squared: " << getRSquared(temp) << std::endl;
    std::cout << "Slope:     " << getSlope( temp ) << std::endl;
    std::cout << "Intercept: " << getIntercept( temp ) << std::endl;
}



/**
 * =================================
 * === FUNCTIONS IMPLEMENTATIONS ===
 * =================================
*/
uint16_t secondsElapsed( void ) {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - program_start).count();
}

void printVectorFloats( const std::vector<float>& the_vector ) {
    std::cout << the_vector.size();
    //for ( const float& val : the_vector )
    //    std::cout << val << "; ";
    std::cout << "\n";
}
