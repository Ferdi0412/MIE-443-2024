// Import guard -> helps when compiling...
#ifndef LIN_APPROX_CPP
#define LIN_APPROX_CPP

#include "lin_approx.hpp"

#include <limits>
#include <cmath>

#ifdef DEBUG_LIN_APPROX
// FOR DEBUGGING!
#include <iostream>
#endif

#ifndef RAD2DEG
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#endif

#ifndef DEG2RAD
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
#endif

/**
 * Intended to be private...
 * _invalidLinApprox returns a value for R-squared, to indicate that the operation failed...
*/
float _invalidLinApprox( void ) { return std::numeric_limits<float>::infinity(); }

/**
 * Intended to be private...
 * _invalidScanElement returns true if a scan element is "invalid"
*/
bool _invalidScanElement( float scan_element ) {
    return std::isnan(scan_element);
    //return scan_element == std::numeric_limits<float>.infinity();
}



/**
 * getMean returns the average distance in a segment of the laser_scan
 *
 * @param input_vector
 * @param start_index
 * @param end_index
 * @returns infinity if no valid, or value if average retrieved
*/
float getMean( const laser_scan_t& input_vector, unsigned int start_index, unsigned int end_index ) {
    double sum_y = 0, n_elements = 0;

    if ( (end_index > input_vector.size()) || (start_index >= end_index) || (start_index >= input_vector.size()) )
        return std::numeric_limits<float>::infinity();

    for ( unsigned int i = 0; i < (end_index - start_index); i++ ) {
        sum_y += input_vector[start_index + i];
        n_elements++;
    }

    if ( n_elements == 0 )
        return std::numeric_limits<float>::infinity();

    return (float) (sum_y / n_elements);
}



/**
 * linearApproximation will calculate a linear approximation for the input scan vector
 *
 * NOTE: No accounting for any overflow of variables...
 *
 * @param input_vector     the scan vector
 * @param start_index      the first index to approximate -> Used as coordinate 0 -> where intercept occurs...
 * @param end_index        the first index to not approximate (this element is NOT included)...
 * @param angle_increments the angle between each point in the laser scan, to adjust for the width of the scan (use 0 to ignore) [deg]
 *
 * @returns a linearApproximation object -> use other functions to retrieve the data from it...
*/
lin_approx_t linearApproximation( const laser_scan_t& input_vector, unsigned int start_index, unsigned int end_index, double angle_increments ) {
    double n_elements, approx_n, sum_x, sum_y, sum_x_squared, sum_xy,
          x_mean, y_mean, mean_squared_error,
          slope, intercept, width_per_increment;
    unsigned int start_offset;

    #ifdef DEBUG_LIN_APPROX
    std::cout << "DEBUG: " << (end_index > input_vector.size()) << "; " << (start_index >= end_index) << "; " << (start_index >= input_vector.size()) << std::endl;
    #endif

    // Check if any input parameters are invalid...
    if ( (end_index > input_vector.size()) || (start_index >= end_index) || (start_index >= input_vector.size()) )
        return lin_approx_t( 0., 0., _invalidLinApprox() );

    #ifdef DEBUG_LIN_APPROX
    std::cout << "DEBUG: " << "Passed initial test!" << std::endl;
    #endif

    // Set aggregating variables to 0
    n_elements         = 0;
    sum_x              = 0;
    sum_y              = 0;
    sum_x_squared      = 0;
    sum_xy             = 0;
    mean_squared_error = 0;
    approx_n           = end_index - start_index;
    start_offset       = start_index;

    // Calculate aggregates...
    for ( unsigned int i = 0; i < approx_n; i++ ) {
        float curr_element;

        // If encountered an invalid element, skip it...
        if ( _invalidScanElement(input_vector[ start_offset + i ]) ) continue;

        #ifdef DEBUG_LIN_APPROX
        std::cout << "DEBUG: " << start_offset + i << "; " << input_vector[ start_offset + i] << std::endl;
        #endif

        // Count elements aggregated over
        n_elements ++;

        // Retrieve element from vector
        curr_element = input_vector[ start_offset + i ];

        // Update variables for slope computation
        sum_x  += i;
        sum_y  += curr_element;
        sum_xy += curr_element * i;
        sum_x_squared += i * i;
    }

    // Check that elements were actually aggregated over...
    if ( n_elements <= 0 ) return lin_approx_t( 0., 0., _invalidLinApprox() );

    // Guard against divide by 0
    if ( (sum_x_squared - sum_x * x_mean) == 0 ) return lin_approx_t( 0., 0., _invalidLinApprox() );

    x_mean = sum_x / n_elements;
    y_mean = sum_y / n_elements;

    slope     = ( sum_xy - sum_x * y_mean ) / ( sum_x_squared - sum_x * x_mean );
    intercept = y_mean - slope * x_mean;

    // Calculate mean squared error
    for ( unsigned int i = 0; i < approx_n; i++ ) {
        float curr_element, prediction;

        // If encountered an invalid element, skip it...
        if ( _invalidScanElement(input_vector[ start_offset + i ]) ) continue;

        curr_element = input_vector[ start_offset + i ];
        prediction   = (slope * i) + intercept;

        mean_squared_error += pow(curr_element - prediction, 2) / n_elements;
    }

    // Adjust span for actual span of scan
    if ( angle_increments != 0. ) {
        angle_increments = fabs(angle_increments);
        width_per_increment = tan(DEG2RAD(angle_increments)) * (intercept + slope * approx_n); // Get average distance times tan(angle) (o/a * a)
        slope /= width_per_increment;
    }

    // Typecast each from double to float...
    return lin_approx_t( (float) slope, (float) intercept, (float) mean_squared_error );
}


/**
 * checkApproximationError will return 0 if approximation could be made, or a value greater than 0 if it could not (ie. error occured)...
 *
 * @param linear_object the object returned from linear_approximation
 * @returns 0 if no errors, >0 if error occured
*/
int checkApproximationError( const lin_approx_t& linear_object ) {
    return std::get<2>(linear_object) == _invalidLinApprox();
}


/**
 * isStraightLine will return 1 if the "line" is straight enough
 *
 * @param linear_object the object returned from linear_approximation
 * @param acceptable_deviation the threshold for determining if the line is straight
 * @returns true if straight enough, otherwise false
*/
bool isStraightLine( const lin_approx_t& linear_object, float acceptable_deviation ) {
    // If any errors, return false
    if ( checkApproximationError(linear_object) ) return false;

    // If calculated R-Squared (from linear_object) is greater than threshold r_squared, return true
    return std::get<2>(linear_object) <= acceptable_deviation;
}


/**
 * getSlope will return the slope of the line
 *
 * @param linear_object the object returned from linear_approximation
 * @returns the slope
*/
float getSlope( const lin_approx_t& linear_object ) {
    return std::get<0>(linear_object);
}


/**
 * getIntercept will return the slope of the line
 *
 * @param linear_object the object returned from linear_approximation
 * @returns the intercept
*/
float getIntercept( const lin_approx_t& linear_object ) {
    return std::get<1>(linear_object);
}


/**
 * getMeanSquaredError returns the mean squared error...
*/
float getMeanSquaredError( const lin_approx_t& linear_object ) {
    return std::get<2>(linear_object);
}

#endif
