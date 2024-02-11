// Import guard -> helps when compiling...
#ifndef LIN_APPROX_HPP
#define LIN_APPROX_HPP

#include <tuple>
#include <vector>

typedef std::vector<float> laser_scan_t;
typedef std::tuple<float, float, float> lin_approx_t;



/**
 * linearApproximation will calculate a linear approximation for the input scan vector
 *
 * NOTE: No accounting for any overflow of variables...
 *
 * @param input_vector the scan vector
 * @param start_index  the first index to approximate -> Used as coordinate 0 -> where intercept occurs...
 * @param end_index    the last index to approximate
 *
 * @returns a linearApproximation object -> use other functions to retrieve the data from it...
*/
lin_approx_t linearApproximation( const laser_scan_t& input_vector, unsigned int start_index, unsigned int end_index );


/**
 * checkApproximationError will return 0 if approximation could be made, or a value greater than 0 if it could not (ie. error occured)...
 *
 * @param linear_object the object returned from linear_approximation
 * @returns 0 if no errors, >0 if error occured
*/
int checkApproximationError( const lin_approx_t& linear_object );


/**
 * isStraightLine will return 1 if the "line" is straight enough
 *
 * @param linear_object the object returned from linear_approximation
 * @param acceptable_deviation the threshold for determining if the line is straight
 * @returns true if straight enough, otherwise false
*/
bool isStraightLine( const lin_approx_t& linear_object, float acceptable_deviation );


/**
 * getSlope will return the slope of the line
 *
 * @param linear_object the object returned from linear_approximation
 * @returns the slope
*/
float getSlope( const lin_approx_t& linear_object );


/**
 * getIntercept will return the slope of the line
 *
 * @param linear_object the object returned from linear_approximation
 * @returns the intercept
*/
float getIntercept( const lin_approx_t& linear_object );


#endif
