#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <auxilliary.h>
#include <chrono>

#include "parin-functions.cpp"
#include "auxilliary.h"

int main(int argc, char** argv) {
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

    /* === TARGET LOCATING USING AUXILLIARY_H === */
    // The following must be run to setup the functions: location_facing_box (plus some more functions from auxilliary.h)
    initialize_boxes_navigation( n, boxes, robotPose );

    // Setup parin-functions::match_function
    imagePipeline.setMatchFunction(&match_function);
    initialize_feature_detector(boxes.templates);

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    std::vector<bool> path_to_box; 

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300 && (!all_found())) {
        ros::spinOnce();

        for ( size_t i = 0; i < boxes.coords.size(); i++ ) {
        // Additional [optional] parameters for {location_facing_box}:
        // 1. distance_from <float> - distance from robot to box
        // 2. delta_phi <float> - angle offset from the image - Imagine a radius around image, that's how this function positions the robot
        //    NOTE: If you want to get to the same position, but with a different orientation, DON'T use delta_phi, rather edit the Navigation::moveToGoal input
            
            
            // only go through boxes that have not yet been found
            if (has_been_found(i)){
                path_to_box.push_back(true);
                continue;
            }

            // Get the posiiton of the box
            SimplePose next_target = location_facing_box(i);

            // Check for a valid path from your location to the box
            if (check_for_plan(next_target)){
                // If valid path found, move to the next_target
                Navigation::moveToGoal( next_target.x, next_target.y, next_target.phi );
                std::cout << "=== {" << i << "} ===\n";
                 
                // Fetch the latest kinect image
                int template_id = try_match_image(i);
                if (template_id < 0)
                    path_to_box.push_back(false);
                else
                    path_to_box.push_back(true);
                ros::Duration(0.01).sleep();
            }
            else
                path_to_box.push_back(false);
        } 

        for ( size_t i = 0; i < path_to_box.size(); i++ ) {
            if ( path_to_box[i] )
                continue;
            // Mess around with location_facing_box(...) params...
            SimplePose next_target = location_facing_box(i, 0.3, 0.1);
            Navigation::moveToGoal( next_target.x, next_target.y, next_target.phi );
            if (has_been_found(i)){
                int template_id = try_match_image(i);
                f (template_id < 0)
                    path_to_box.push_back(false);
                else
                    path_to_box.push_back(true);
                ros::Duration(0.01).sleep();
            }
        }
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        
    }
    return 0;
}

int try_match_image( size_t box_coord_i ) {
    // Make sure image is up to date
    ros::spinOnce();
    
    // Run feature matching, to get the best match
    int template_id = imagePipeline.getTemplateID(boxes);
    
    // If template wasn't matched....
    if ( template_id < 0 ) {
        std::cout << "Could not match box " << box_coord_i << std::endl;
    }
    else {
        // Add how to store which box is which
        boxes.template_ids[i] = template_id;
        mark_as_found( box_coord_i );
    }
    return template_id;
}