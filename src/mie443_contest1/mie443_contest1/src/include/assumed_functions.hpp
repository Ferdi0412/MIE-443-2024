/**
 * This file lays out the majority of the "higher-level" functions that we use.
 *
 * NOTE that the majority of them return integers, this is to prevent any errors where we forget to 'catch' any
 * of the potential exceptions thrown by the robot class.
 *
 * As a result we have the "MOVEMENT RESULTS" below which are constants used to check what happened in the higher level command....
*/


#ifndef ASSUMED_FUNCTIONS_HPP
#define ASSUMED_FUNCTIONS_HPP


// Robot class is input to majority of the functions here...
#include "../robot.cpp"


/**
 * MOVEMENT RESULTS
*/
#define REACHED_TARGET_LEFT    -1 // wallFollow success
#define REACHED_TARGET_RIGHT   -2 // wallFollow success
#define REACHED_TARGET_CENTER  -3 // wallFollow success
#define REACHED_TARGET          0
#define WALL_BUMPED             1 // GREATER THAN 0 INDICATES ERROR/EXCEPTIONS
#define WALL_IN_FRONT           2
#define WALL_NOT_FOUND          3
#define NO_MOVE                 4



/**
 * getWallAngleFromLaserScan returns the angle the robot must turn to be NORMAL to a wall in the center of it's laser-scan
 *
 * @param laser_scan the laser scan vector
 * @returns an angle in range [-180, 180] if successful, or inf if unsuccessful (ie. no wall in front, or error)
*/
float getWallAngleFromLaserScan( Team1::Robot& robot );
float getWallAngleFromLaserScanNonStraight( Team1::Robot& robot );



/**
 * turnRobotBy will turn the robot by a given amount
 *
 * @param robot the robot object
 * @param angle the clockwise-positive angle
 * @returns REACHED_TARGET if no errors, WALL_BUMPED if bumped a wall
*/
int turnRobotBy( Team1::Robot& robot, double angle )



/**
 * moveForwardsBy will move the robot forwards
 *
 * @param robot           the robot object
 * @param target_distance the distance to travel forwards
 * @param wall_distance   the distance from a wall at which it should stop
 * @returns REACHED_TARGET if target distance travelled, WALL_BUMPED if a wall was bumped, or WALL_IN_FRONT if something in laser scan
*/
int moveForwardsBy( Team1::Robot& robot, double target_distance, float wall_distance );



/**
 * getRandomValue returns a random value between min_val and max_val
 *
 * @param min_val
 * @param max_val
 * @returns value between the min_val and max_val
*/
float getRandomValue( float min_val, float max_val );




/**
 * Scanmotion does a 180 degree scan and moves in that direction until 0.2 away from the wall
 *
 * @param robot the robot object
*/
int scanMotion(Team1::Robot& robot);




// Enum for use with followWall
enum wallDirectionEnum { left, right, any };

/**
 * wallFollow makes the robot follow a straight segment of a wall detected in front of the robot
 *
 * @param robot the robot object
 * @param wall_direction the side of the robot on which to keep the wall (not used anywhere...)
 * @returns WALL_BUMPED or REACHED_TARGET
*/
int wallFollow( Team1::Robot& robot, wallDirectionEnum wall_direction );



/**
 * randomMotion rotates and moves in a random direction with an orientation between min_angle and max_angle
 *
 * @param robot the robot object
 * @param min_angle the minimum acceptable clockwise-positive rotation
 * @param max_angle the maximum acceptable clockwise-positive rotation
 * @returns REACHED_TARGET on success, WALL_BUMPED on wall bump
*/
int randomMotion( Team1::Robot& robot, double min_angle, double max_angle );



/**
 * rotateAfterBumper will move the robot after it has bumped into a wall (bumper is still triggered)...
 *
 * @param robot the robot object
 * @returns REACHED_TARGET if successful, WALL_BUMPED if bumped into wall when rotating...
*/
int rotateAfterBumper( Team1::Robot& robot );



/**
 * checkIfFacingCorner will return true if the robot is facing a 90 (or so) degree corner in the maze
*/
bool checkIfFacingCorner( Team1::Robot& robot , int MIN_DISTANCE);



/**
 * scanForArea will turn the robot to face the biggest opening it detects
 *
 * @param robot the robot object
 * @returns angle to travel in -> DO NOT USE IN MAIN, instead use scanMotion
*/
int scanForArea( Team1::Robot& robot );



/**
 * wallInFront check if there is a straight wall segment directly in front of the robot
 *
 * Checks if there is a wall detected, using the lin_approx functions (using a Mean Squared Error threshold for straightness)
 * no distance checks or angle constraints held, just straightness of the detected surface.
 *
 * @param robot object
 * @returns true if a straight wall is found, else false
*/
bool wallInFront( Team1::Robot& robot );



/**
 * distanceToWall in front of robot
 *
 * @param robot object
 * @returns the average distance to a straight wall segment directly in front of the robot
*/
double distanceToWall( Team1::Robot& robot );



/**
 * emptyInFront return true if insufficient points to check for wall in laser scan...
*/
bool emptyInFront( Team1::Robot& robot );



#endif
