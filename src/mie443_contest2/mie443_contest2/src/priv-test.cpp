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

// The following is a standard "system" import
#include <chrono>



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

    int my_callback( const cv::Mat& kinect_img, const std::vector<cv::Mat>& logos ) {
        // Image Processsing LOGIC
        return -1;
    }

    // imagePipeline.setBoxMatcher(&my_callback)

    /**
     * === MAIN BODY ===
    */
    // Execute strategy.
    while(ros::ok() && mainTimerSecondsElapsed() <= CONTEST_TIME) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        imagePipeline.getTemplateID_v2(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}
