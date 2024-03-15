/**
 * This file is intended for private testing. It will NOT be commited to GitHub.
 * Any functions used here, that are intended to be used elsewhere, should be in a seperate file.
 *
 * NOTE: If you want to stop the code, and display that an error occured, you have 2 options:
 * 1. `return` from the main function. A return of 0 indicates successful completion, any other value indicates an error occured.
 * 2. `exit(0)` from anywhere. This will kill main, and is the same as a line of `return 0` in main.
*/

/**
 * ===================
 * === DEFINITIONS ===
*/
#define CONTEST_TIME 300

/**
 * ===============
 * === IMPORTS ===
*/
// The following files are found in "../include/" directory
#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <auxilliary.h>

#include "robot_plan.h"

// The following is a standard "system" import
#include <chrono>

// OpenCV
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>


/**
 * ============
 * === MAIN ===
*/
int main(int argc, char** argv) {
    /**
     * === ROS SETUP ===
    */
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);

    // Initialize box coordinates and templates
    Boxes boxes;
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: "
                  << boxes.coords[i][2] << std::endl;
    }

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // contest count down timer
    mainTimerStart();

    /**
     * === SETUP ===
     * Add any setup here...
    */

    /* === TARGET LOCATING USING AUXILLIARY_H === */
    // The following must be run to setup the functions: location_facing_box (plus some more functions from auxilliary.h)
    initialize_boxes_navigation( n, boxes, robotPose );

    /**
     * === "MAIN" ===
     * Just move to each box, and display an image of them...
    */
    for ( size_t i = 0; i < boxes.coords.size(); i++ ) {
        // Additional [optional] parameters for {location_facing_box}:
        // 1. distance_from <float> - distance from robot to box
        // 2. delta_phi <float> - angle offset from the image - Imagine a radius around image, that's how this function positions the robot
        //    NOTE: If you want to get to the same position, but with a different orientation, DON'T use delta_phi, rather edit the Navigation::moveToGoal input
        SimplePose next_target = location_facing_box(i);

        // Check if there exists a valid path to the next_target...
        if ( !check_for_plan(next_target) ) {
            std::cout << "Could not locate box {" << i << "}!\n";
            continue;
        }

        // If valid path found, move to the next_target
        Navigation::moveToGoal( next_target.x, next_target.y, next_target.phi );
        std::cout << "=== {" << i << "} ===\n";

        // Fetch the latest kinect image
        ros::spinOnce();
        cv::Mat img = imagePipeline.getKinectImage();

        // If a valid kinect image found, display it...
        if ( img.empty() || img.rows <= 0 || img.cols <= 0 )
            ;
        else {
            cv::imshow("Image...", img);
            cv::waitKey(100);
        }

        // Wait 0.2 seconds before going to next target...
        ros::Duration(0.2).sleep();  
    }

    return 0;
}
