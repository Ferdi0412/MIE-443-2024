#ifndef OPENING_NAV_CPP
#define OPENING_NAV_CPP

/** STD IMPORTS */
#include <utility> // std::pair
#include <limits>  // std::numeric_limits<T>::infinity()
#include <vector>  // std::vector<T>

/** ROBOT CLASS */
#include "robot.cpp"



/**
 * =============================
 * === FUNCTION DECLARATIONS ===
 * =============================
*/

/**
 * applyConvolution1D will apply a simple kernel to laser_scan
 *
 * @param laser_scan reference to the laser_scan vector from the robot
 * @param kernel     the kernel to use in applying the convolution
 * @returns a new vector that is the result of the convolution
*/
std::vector<float> applyConvolution1D( const std::vector<float>& laser_scan, const std::vector<float>& kernel );
std::vector<float> applyConvolution1D( const std::vector<float>& laser_scan, unsigned int avg_across_n );

/**
 * searchFarthestPoint will return the index of the farthest point in laser_scan
 *
 * @param laser_scan    reference to the laser_scan vector from the robot
 * @param greatest_dist the value of the deepest point
 * @returns std::numeric_limits<int>::infinity() in case of no value found/error
*/
int searchFarthestPoint( const std::vector<float>& laser_scan );
int searchFarthestPoint( const std::vector<float>& laser_scan, float greatest_dist );

/**
 * searchWidestOpening will return the start and end indexes of the widest opening in laser_scan
 *
 * @param laser_scan   reference to the laser_scan vector from the robot
 * @param min_distance the minimum distance to define as an opening
 * @param min_width    the minimum width to define as an opening
*/
std::pair<unsigned int, unsigned int> searchWidestOpening( const std::vector<float>& laser_scan, float min_distance, unsigned int min_width );



/**
 * ===============================
 * === FUNCTION IMPLEMENTATION ===
 * ===============================
*/
std::vector<float> applyConvolution1D( const std::vector<float>& laser_scan, const std::vector<float>& kernel ) {
    long lower_j = -(kernel.size() / 2), upper_j = kernel.size() / 2;
    std::vector<float> convoluted_scan( laser_scan.size() ); // Make copy of laser_scan
    for ( long i = 0; i < convoluted_scan.size(); i++ ) {
        convoluted_scan[i] = 0;
        for ( long j = lower_j; j <= upper_j; j++ ) {
            long idx;
            if ( (i + j) < 0 )
                idx = 0;
            else if ( (i + j) >= laser_scan.size() )
                idx = laser_scan.size() - 1;
            else
                idx = i + j;
            convoluted_scan[i] += laser_scan[idx] * kernel[j];
        }
    }
    return convoluted_scan;
}

std::vector<float> applyConvolution1D( const std::vector<float>& laser_scan, unsigned int avg_across_n ) {
    float one_over_n = 1 / ((float) avg_across_n);
    std::vector<float> kernel( avg_across_n );
    for ( unsigned int i = 0; i < kernel.size(); i++ ) {
        kernel[i] = one_over_n;
    }
    return applyConvolution1D( const std::vector<float>& laser_scan, kernel );
}

int searchFarthestPoint( const std::vector<float>& laser_scan ) {
    unsigned int index_found = std::numeric_limits<unsigned int>::infinity();
    float greatest_dist = std::numeric_limits<float>::lowest();
    for ( unsigned int i; i < laser_scan.size(); i++ ) {
        if ( laser_scan[i] > greatest_dist ) {
            greatest_dist = laser_scan[i];
            index_found = i;
        }
    }
    return index_found;
}

int searchFarthestPoint( const std::vector<float>& laser_scan, float greatest_dist ) {
    // If multiple points with same value, return first instance
    for ( unsigned int i; i < laser_scan.size(); i++ ) {
        if ( laser_scan[i] == greatest_dist ) return i;
    }
    return std::numeric_limits<unsigned int>::infinity();
}

#endif
