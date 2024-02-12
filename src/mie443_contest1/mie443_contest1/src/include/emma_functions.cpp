#ifndef EMMA_FUNCTIONS_CPP
#define EMMA_FUNCTIONS_CPP

#include "assumed_functions.hpp"
#include "lin_approx.hpp"


// STD imports
#include <vector>
#include <cmath>
#include <chrono>
#include <stdint.h>
#include <stdio.h>

// ROS imports
#include <ros/console.h>
#include "ros/ros.h"

// Team1::Robot import
#include "../robot.cpp"

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

int getRandomValue(double  minVal,double&  maxVal){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(-minVal,maxVal);
    std::cout << "Random Angle is: ",(int) distr(gen);
    return (int) distr(gen);
}

int rotateAfterBumper(Team1::Robot& robot){ //tests
try{
            if (robot.getBumperRight() == true){
                robot.moveForwards(-0.25,0.2);
                robot.rotateClockwiseBy(60, -30);
            }
            else if (robot.getBumperLeft() == true){
                robot.moveForwards(-0.25,0.2);
                robot.rotateClockwiseBy(60, 30);
            }
            else {
            robot.moveForwards(-0.25,0.2);
            robot.rotateClockwiseBy(80,180);
}
}
catch (BumperException)
{
    return WALL_BUMPED;
}
    robot.spinOnce();
      // Otherwise return REACHED_TARGET when all movements complete
    return REACHED_TARGET;
}

int scanForArea(Team1::Robot& robot){
    ROS_INFO("SCANNING");
    robot.spinOnce();
    int maxArr[6];
    int bestDir;
    int scanArray[6] = {-90,30, 30, 60, 30, 30};
    double longLength = 0;
    for (unsigned int i=0; i<6;  i++){
        robot.rotateClockwiseBy(60, scanArray[i]);
        robot.spinOnce();
        maxArr[i] = printVectorAvg(robot.getRanges());
        }
    int n = 0;
    for (unsigned int n=0;n<6; n++){
        if (longLength < maxArr[n]){
            longLength = maxArr[n];
            if (n > 2){
                bestDir = (5-n) *-30;}
            
            else {bestDir = (n * 30) - 180;
            }

            }
        }
    
    ROS_INFO("Direction to go: %d", bestDir);
    return bestDir;

}

int randomMotion(Team1::Robot& robot, double minValue, double maxValue){
        try {
            robot.rotateClockwiseBy(60, getRandomValue(-minValue,maxValue));
            robot.moveForwards(0.25,printVectorFloats(robot.getRanges()) - 0.5 );
        }
        catch (BumperException){
            rotateAfterBumper(robot);
            return WALL_BUMPED;
        }
    return REACHED_TARGET;
    }

int randomBias(Team1::Robot& robot){
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
    robot.stopMotion();
    return WALL_BUMPED;
    }

#endif