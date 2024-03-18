#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <auxilliary.h>
#include <chrono>

#include "parin-functions.cpp"
#include "auxilliary.h"


bool try_match_image( size_t box_index, ImagePipeline& image_pipeline, Boxes& boxes  );

bool try_move_to_box( size_t box_index );


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

    SimplePose start_pose(robotPose);

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300 ) {
        ros::spinOnce();

        if ( all_found() ) {
            // Try to return to home... and then exit the main loop
            if ( check_for_plan(start_pose) )
                Navigation::moveToGoal(start_pose.x, start_pose.y, start_pose.phi);
            break;
        }

        for ( size_t i = 0; i < boxes.coords.size(); i++ ) {
            // Skip any that have been found
            if ( has_been_found(i) )
                continue;
            
            if ( try_move_to_box(i) )
                if ( try_match_image(i, imagePipeline, boxes ) ) {
                    cv::imshow("Template matched", boxes.templates[i]);
                    cv::waitKey(10);
                    ros::Duration(1.).sleep();
                }
        }
        
    }
    return 0;
}







bool try_match_image( size_t box_index, ImagePipeline& image_pipeline, Boxes& boxes ) {
    int image_id = image_pipeline.getTemplateID(boxes);
    if ( image_id > -1 ) {
        mark_as_found(box_index, image_id);
        return true;
    }
    else
        return false;
}


bool try_move_to_box( size_t box_index ) {
    SimplePose facing_box = location_facing_box( box_index );
    
    // Try move to position directly ahead of box, facing it
    if ( check_for_plan( facing_box ) ) {
        Navigation::moveToGoal( facing_box.x, facing_box.y, facing_box.phi );
        return true;
    }

    return false;
}