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
#include <random>
#include <iostream>

// ROS imports
#include <ros/console.h>
#include "ros/ros.h"

// Team1::Robot import
#include "../robot.cpp"

#define EMMA_LIN_SPEED 0.15
#define EMMA_ROT_SPEED 45

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
    std:: cout << laserAvg;
    return laserAvg;
}

double printLaserAvg(Team1::Robot& robot, const std::vector<float>& the_vector){
    const std::vector<float> laser_ranges = robot.getRanges();
    // printVectorFloats(laser_ranges);
    if (!laser_ranges.empty()) {

        const float middle_value = laser_ranges[laser_ranges.size() / 2];
        const float right_value = laser_ranges[0];
        const float left_value = laser_ranges[laser_ranges.size() - 1];

        float laserAverage = (middle_value + right_value + left_value)/3;

        return laserAverage;
    }
}

float getRandomValue(float  minVal,float  maxVal){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> distr(-minVal,maxVal);
    float randomval = distr(gen);
    std:: cout << "random value is: " << randomval;
    return randomval;
}

int rotateAfterBumper(Team1::Robot& robot){ //tests
try{
            if (robot.getBumperRight() == true){
                robot.moveForwards(-0.25,0.2);
                robot.rotateClockwiseBy(EMMA_ROT_SPEED, -30);
            }
            else if (robot.getBumperLeft() == true){
                robot.moveForwards(-0.25,0.2);
                robot.rotateClockwiseBy(EMMA_ROT_SPEED, 30);
            }
            else {
            robot.moveForwards(-0.25,0.2);
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
    try {
    ROS_INFO("SCANNING");
    robot.spinOnce();
    int bestDir = 0;
    float fourtyfivedegreeLength = 0;
    float ninetydegreeLength = 0;
    robot.rotateClockwiseBy(EMMA_ROT_SPEED, -45);
    robot.spinOnce();
    fourtyfivedegreeLength = printLaserAvg(robot,robot.getRanges());
     if (fourtyfivedegreeLength > 2)
        {
        ROS_INFO("Distance > 2");
            return REACHED_TARGET;
            }
    robot.rotateClockwiseBy(EMMA_ROT_SPEED, 90);
    robot.spinOnce();
    ninetydegreeLength = printLaserAvg(robot,robot.getRanges());
    ROS_INFO("45 degree is: %f", fourtyfivedegreeLength);
    ROS_INFO("90 degree is: %f", ninetydegreeLength);
    if (fourtyfivedegreeLength > ninetydegreeLength){
         bestDir = -90;
    }
    else {bestDir  = 0;
    }
    ROS_INFO("best dir is: %i", bestDir);
    return bestDir;
    }
catch (BumperException)
{
    //rotateAfterBumper(robot);
    return WALL_BUMPED;
}

}

int scanForAreaOld(Team1::Robot& robot){
    try {
    ROS_INFO("SCANNING");
    robot.spinOnce();
    float maxArr[2];
    int bestDir = 0;
    int scanArray[2] = {-45,90};
    float longLength = 0;
    int dir = 0;;
    for (int i=0; i<2;  i++){
        ROS_INFO("i is %i", i);
        ROS_INFO("degree rot is %i", scanArray[i]);
        robot.rotateClockwiseBy(EMMA_ROT_SPEED, scanArray[i]);
        robot.spinOnce();
        maxArr[i] = printLaserAvg(robot,robot.getRanges());
        ROS_INFO("output: %f",maxArr[i]);
        if (maxArr[i] > 2)
        {
        ROS_INFO("Distance > 2");
            return REACHED_TARGET;
            }
        }
    for (int n=0;n<2; n++){
        if (longLength < maxArr[n]){
        ROS_INFO("Length is: %f", longLength);
        ROS_INFO("Array max is: %f",maxArr[n]);
            longLength = maxArr[n];
             dir = n;
            }
            else{std:: cout << "n  is: " << n << "avg distance not longer";}
        }

            if (dir == 0){
                bestDir = -90;
                ROS_INFO("best dir is: %i", bestDir);
                }

            else {bestDir = 0;
            ROS_INFO("best dir is: %i", bestDir);
            }

    ROS_INFO("Direction to go: %d", bestDir);
    return bestDir;
    }
catch (BumperException)
{
    //rotateAfterBumper(robot);
    return WALL_BUMPED;
}

}

int randomMotion(Team1::Robot& robot, double minValue, double maxValue){
        try {
            float degreeVal = getRandomValue(-minValue,maxValue);
            ROS_INFO("Degree is: %f",degreeVal);
            robot.rotateClockwiseBy(EMMA_ROT_SPEED, degreeVal);
            robot.moveForwards(EMMA_LIN_SPEED ,printLaserAvg(robot,robot.getRanges()) - 0.2);
        }
        catch (BumperException){
            //rotateAfterBumper(robot);
            return WALL_BUMPED;
        }
    return REACHED_TARGET;
    }

int scanMotion(Team1::Robot& robot){
        try {
            int degreeRot = scanForArea(robot);
            robot.rotateClockwiseBy(EMMA_ROT_SPEED, degreeRot);
            robot.moveForwards(EMMA_LIN_SPEED, printLaserAvg(robot,robot.getRanges()) -  0.2);
        }
        catch (BumperException){
            //rotateAfterBumper(robot);
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
            robot.moveForwards(EMMA_LIN_SPEED, printVectorFloats(robot.getRanges()) - 0.5 );
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
