/**
 * This lays out the format of the functions we discussed in our earlier meeting
*/
#ifndef ASSUMED_FUNCTIONS_HPP
#define ASSUMED_FUNCTIONS_HPP


#include "../robot.cpp"

// MOVEMENT RESULTS
#define REACHED_TARGET_LEFT    -1 // wallFollow success
#define REACHED_TARGET_RIGHT   -2 // wallFollow success
#define REACHED_TARGET_CENTER  -3 // wallFollow success
#define REACHED_TARGET          0
#define WALL_BUMPED             1 // GREATER THAN 0 INDICATES ERROR/EXCEPTIONS
#define WALL_IN_FRONT           2



/**
 * getWallAngleFromLaserScan returns the angle the robot must turn to be NORMAL to a wall in the center of it's laser-scan
 *
 * @param laser_scan the laser scan vector
 * @returns an angle in range [-180, 180] if successful, or inf if unsuccessful (ie. no wall in front, or error)
*/
float getWallAngleFromLaserScan( Team1::Robot& robot ); // Ajeya/ferdi



/**
 * turnRobotBy will turn the robot by a given amount
 *
 * @param robot the robot object
 * @param angle the clockwise-positive angle
 * @returns REACHED_TARGET if no errors, WALL_BUMPED if bumped a wall
 * @throws NOTHING -> kill all exceptions
*/
int turnRobotBy( Team1::Robot& robot, double angle ); // DONE



/**
 * moveForwardsBy will move the robot forwards
 *
 * @param robot           the robot object
 * @param target_distance the distance to travel forwards
 * @param wall_distance   the distance from a wall at which it should stop
 * @returns REACHED_TARGET if target distance travelled, WALL_BUMPED if a wall was bumped, or WALL_IN_FRONT if something in laser scan
 * @throws NOTHING -> kill all exceptions
*/
int moveForwardsBy( Team1::Robot& robot, double target_distance, float wall_distance ); // DONE



/**
 * getRandomValue returns a random value between min_val and max_val
 *
 * @param min_val
 * @param max_val
 * @returns value between the min_val and max_val
*/
float getRandomValue( float min_val, float max_val ); // EMMA



// Enum for use with followWall
enum wallDirectionEnum { left, right, any };

/**
 * wallFollow makes the robot follow a straight segment of a wall detected in front of the robot
 *
 * @param robot the robot object
 * @param wall_direction the side of the robot on which to keep the wall
 * @returns .... TBD
*/
int wallFollow( Team1::Robot& robot, wallDirectionEnum wall_direction ); // PARIN



/** === SCRATCH randomMotion(robot)... === */
// /**
//  * randomMotion will complete a linear motion in a random direction
//  *
//  * @param robot the robot object
//  * @returns ... TBD
// */
// int randomMotion( Team1::Robot& robot ); // EMMA



/**
 * randomMotion [OVERLOAD] same as randomMotion, but takes a min_angle and max_angle input...
 *
 * @param robot the robot object
 * @param min_angle the minimum acceptable clockwise-positive rotation
 * @param max_angle the maximum acceptable clockwise-positive rotation
 * @returns .... SAME as default randomMotion
*/
int randomMotion( Team1::Robot& robot, double min_angle, double max_angle ); // EMMA



/**
 * rotateAfterBumper will move the robot after it has bumped into a wall (bumper is still triggered)...
 *
 * @param robot the robot object
 * @returns REACHED_TARGET if successful, WALL_BUMPED if bumped into wall when rotating...
*/
int rotateAfterBumper( Team1::Robot& robot ); // EMMA



/**
 * checkIfFacingCorner will return true if the robot is facing a 90 (or so) degree corner in the maze
*/
bool checkIfFacingCorner( Team1::Robot& robot ); // AJEYA



/** === SCRATCHED === */
// /**
//  * checkIfFacingRoundObject will return true if the robot is facing a round obstacle...
// */
// bool checkIfFacingRoundObject( Team1::Robot& robot );



/**
 * scanForArea will turn the robot to face the biggest opening it detects...
*/
int scanForArea( Team1::Robot& robot ); // EMMA



/**
 * wallInFront
*/
bool wallInFront( Team1::Robot& robot ); // AJEYA



/**
 * distanceToWall in front of robot
*/
double distanceToWall( Team1::Robot& robot ); // AJEYA



/**
 * emptyInFront
*/
bool emptyInFront( Team1::Robot& robot ); // FERDI



#endif
