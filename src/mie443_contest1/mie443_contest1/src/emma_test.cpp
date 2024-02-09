// STD imports
#include <vector>
#include <cmath>
#include <chrono>
#include <stdint.h>
#include <stdio.h>
#include <random>

// ROS imports
#include <ros/console.h>
#include "ros/ros.h"

// Team1::Robot import
#include "robot.cpp"

int randDirection;

/**
 * ==============================
 * === FUNCTIONS DECLARATIONS ===
 * ==============================
*/

// ========= EDIT THE FOLLOWING DEFINITION BELOW =========
/**
 * moveAndScan_example
 *
 * An example function for how to move and scan the robot.
 *
 * @param robot the robot object
 * @param angle the distance to travel [meters]
*/
void moveAndScan_example( Team1::Robot robot, double distance );


// ========= AUXILLIARY =========
/**
 * secondsElapsed
 *
 * @returns number of seconds from program_start
*/
uint16_t secondsElapsed(void);

/**
 * exitIfTimeRunOut
 *
 * Will terminate the program (exit/return) if the program has completed
*/
void exitIfTimeRunOut( void );

/**
 * exitIfTimeRunOut [OVERLOAD]
 *
 * Will terminate the program (exit/return) with exit_code if the program has completed
 *
 * @param exit_code 0 means no error, any other value means program ended with an error (C-standard)
*/
void exitIfTimeRunOut( unsigned int exit_code );



/**
 * =====================
 * === GLOBAL params ===
 * =====================
*/
#define LINEAR_SPEED 0.2
#define ROTATIONAL_SPEED 10

static std::chrono::time_point<std::chrono::system_clock> program_start;

static const unsigned long long program_duration = 10;

double printVectorFloats( const std::vector<float>& the_vector ) {
    // std::cout << the_vector.size();
    double midValue;
    //for ( const float& val : the_vector )
    //    std::cout << val << "; ";
    std::cout << "\n";
    midValue = the_vector[the_vector.size()/2];
    //for ( const float& val : the_vector )
    //    std::cout << val << "; ";
    std::cout << midValue;
    return midValue;
}

double printVectorAvg( const std::vector<float>& the_vector ) {
    // std::cout << the_vector.size();
    float laserAvg;
    float sum;
      // std::cout << the_vector.size()
    for (unsigned int i = 0; i < 640; i++){
        sum = sum + (the_vector[i]);
        }
    laserAvg = sum / 640;
    ROS_INFO("LASER AVG IS PRINTED");
    return laserAvg;
}

int genRandom(){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(-90,90);
    std::cout << "Random direc is: ",(int) distr(gen);
    return (int) distr(gen);
}

void rotateAfterBumper(Team1::Robot& robot){ //tests
            if (robot.getBumperRight() == true){
                robot.moveForwards(-0.25,0.2);
                robot.rotateClockwiseBy(60, -45);
            }
            else if (robot.getBumperLeft() == true){
                robot.moveForwards(-0.25,0.2);
                robot.rotateClockwiseBy(60, 45);
            }
            else {
            robot.moveForwards(-0.25,0.2);
            robot.rotateClockwiseBy(80,genRandom());
}
    robot.spinOnce();
}

int scanForArea(Team1::Robot& robot){
    ROS_INFO("SCANNING");
    robot.spinOnce();
    int maxArr[4];
    int bestDir;
    int scanArray[6] = {-90,-60, 30, 60, 30, 30};
    double longLength = 0;
    for (unsigned int i=0; i<4;  i++){
        robot.rotateClockwiseBy(60, scanArray[i]);
        robot.spinOnce();
        maxArr[i] = printVectorAvg(robot.getRanges());
        }
    int n = 0;
    for (unsigned int n=0;n<4; n++){
        if (longLength > maxArr[n]){
            longLength = maxArr[n];
            if (n > 2){
                bestDir = n *-30;}
            
            else {bestDir = ((5-n) *-30) + 30;
            }

            }
        }
    
    ROS_INFO("Direction to go: %d", bestDir);
    return bestDir;

}

void randomBias(Team1::Robot& robot){
    ROS_INFO("Phase 3.1");
    while (ros::ok){

    robot.spinOnce();
    if (printVectorFloats(robot.getRanges()) > 0.5){
         ROS_INFO("Distance > 0.4");
        std::cout << "distance is";
        std::cout << printVectorFloats(robot.getRanges());
        try {
            robot.moveForwards(0.25,printVectorFloats(robot.getRanges()) - 0.5 );
            ROS_INFO("Moving Forward");
        }
        catch (BumperException){
            ROS_INFO("Caught Bumper");
            rotateAfterBumper(robot);
        }
        }
    else {
         ROS_INFO("Distance is less than 0.4");
        //scanForArea(robot);
        try {
             ROS_INFO("Scanning");
        scanForArea(robot);
        }
        catch (BumperException){
             ROS_INFO("Caught Bumper");
            rotateAfterBumper(robot);
        }
    }
    }
    robot.stopMotion();

    }
}

/**
 * printLaserScan
 * 
 * @param laser_vector the output from the robot.getRanges() method
*/




/**
 * ============
 * === MAIN ===
 * ============
 *
 * @param argc number of params used in starting program (ignore but keep)
 * @param argv string params used when starting program  (ignore but keep)
*/
int main ( int argc, char **argv ) {
    // === SETUP ===
    ROS_INFO("SETUP...\n");

    ros::init(argc, argv, "contest1");

    ros::NodeHandle nh;
    ros::Rate loop_rate(2);
    Team1::Robot robot( nh, 2);

    ros::Duration(0.5).sleep(); // Sleep to ensure is initialized correctly
    robot.spinOnce();
    
    ROS_INFO("Phase 1"); 

    while ( ros::ok() && robot.getRanges().size() == 0 ) {
        robot.spinOnce();
        ROS_INFO("Phase 2");
        ros::Duration(0.5).sleep();
    }

    program_start = std::chrono::system_clock::now();
    ROS_INFO("Phase 3");

    randomBias(robot);
     ROS_INFO("Phase 4");
    // === MAIN ===
    // loop until program_duration [seconds] is reached
    while ( ros::ok() && secondsElapsed() <= program_duration ) {
        // Main stuff here...  

        robot.sleepOnce();
    }


    // === PROGRAM END ===
    robot.stopMotion();
    ROS_INFO("ENDED...\n");
}



/**
 * =================================
 * === FUNCTIONS IMPLEMENTATIONS ===
 * =================================
*/
// ========= EDIT THE FOLLOWING DEFINITION BELOW ========={}}

void moveAndScan_example( Team1::Robot robot, double distance ) {
    double start_x = robot.getX(), start_y = robot.getY();
    robot.jogForwardsSafe( 0.2 ); // Start moving at 0.2 [m/s]

    // Check distance from robot to starting position, until distance (in units of [m]) has been travelled
    while ( (secondsElapsed() << program_duration) && (robot.distanceToPoint( start_x, start_y ) < distance) ) {
        // Update values in robot from subscriptions
        robot.spinOnce();

        // If any bumper has been triggered, robot has collided with a wall...
        // Stop the motion of the robot, and return
        if ( robot.getBumperAny() ) {
            robot.stopMotion();
            return;
        }

        // Do something with the scan data
        std::vector<float> scan_distances = robot.getRanges();
        // ...

        robot.sleepOnce();
    }

    // Once enough distance travelled...
    robot.stopMotion();
}


// ========= AUXILLIARY =========
uint16_t secondsElapsed( void ) {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - program_start).count();
}

void exitIfTimeRunOut( void ) {
    if ( secondsElapsed() > program_duration ) exit(0);
}

void exitIfTimeRunOut( unsigned int exit_code ) {
    if ( secondsElapsed() > program_duration ) exit( exit_code );
}
