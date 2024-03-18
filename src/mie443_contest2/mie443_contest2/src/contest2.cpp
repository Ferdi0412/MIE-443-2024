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

bool not_timedout();

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
    std::cout << "Initializing 1\n";
    initialize_boxes_navigation( n, boxes, robotPose );

    // Setup parin-functions::match_function
    std::cout << "Initializing\n";
    
    imagePipeline.setMatchFunction(&match_function);
    initialize_feature_detector(boxes.templates);

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    // uint64_t secondsElapsed = 0;
    std::vector<bool> path_to_box(boxes.coords.size(), false);

    std::cout << "Storing starting_position\n";

    SimplePose start_pose(robotPose);

    std::cout << "Before loop\n";

    mainTimerStart();

    // Execute strategy.
    while(ros::ok() && not_timedout() ) {
        ros::spinOnce();

        std::cout << "Loop\n";

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

            std::cout << "Running now for box " << i << std::endl;
            
            if ( try_move_to_box(i) ) {
                path_to_box[i] = true;
                if ( try_match_image(i, imagePipeline, boxes ) ) {
                    std::cout << "Displaying matched image..." << std::endl;
                    int template_id;
                    if ( (template_id = get_box_id(i)) > 0 ) {
                        cv::imshow("***** Template matched", boxes.templates[template_id]);
                        cv::waitKey(10);
                        ros::Duration(1.).sleep();
                    }
                }
            } else {
                std::cout << "No path found" << std::endl;
                path_to_box[i] = false;
                continue;
            }

            std::cout << "try_move_to_box done...\n";

            // break;
        }
        
        // Move to starting position
        // if ( check_for_plan(start_pose) ) {
        //     std::cout << "Moving to start position" << std::endl;
        //     Navigation::moveToGoal(start_pose.x, start_pose.y, start_pose.phi);
        //     ros::spinOnce();
        // }

        for (size_t j=0; j < path_to_box.size(); j++){
            if (path_to_box[j])
                continue;
            else{
                if ( has_been_found(j) )
                    continue;

                std::cout << "Running now for box " << j << std::endl;
            
                if ( try_move_to_box(j) ) {
                    path_to_box[j] = true;
                    if ( try_match_image(j, imagePipeline, boxes ) ) {
                        std::cout << "Displaying matched image..." << std::endl;
                        int template_id;
                        if ( (template_id = get_box_id(j)) > 0 ) {
                            cv::imshow("Template matched", boxes.templates[template_id]);
                            cv::waitKey(10);
                            ros::Duration(1.).sleep();
                        }
                    }
                } else {
                    path_to_box[j] = false;
                    continue;
                }

            }
        }
        
    }
    return 0;
}







bool try_match_image( size_t box_index, ImagePipeline& image_pipeline, Boxes& boxes ) {
    int image_id = image_pipeline.getTemplateID(boxes);
    if ( image_id > -1 ) {  
        std::cout << "Marking box " << box_index << " as " << image_id << std::endl;
        mark_as_found(box_index, image_id);
        return true;
    }
    else
        return false;
}


bool try_move_to_box( size_t box_index ) {
    SimplePose facing_box = location_facing_box( box_index );
    
    std::cout << "trying to move to facing_box for box" << box_index << std::endl;
    std::cout << "Facing box coordinates: x = " << facing_box.x << "y= " << facing_box.y <<std::endl;
    // Try move to position directly ahead of box, facing it
    if ( check_for_plan( facing_box ) ) {
        bool success = Navigation::moveToGoal( facing_box.x, facing_box.y, facing_box.phi );
        ros::spinOnce();
        return success;
    } else {
        std::cout << "No plan found for facing_box of box" << box_index << std::endl;
    }

    return false;
}


bool not_timedout() {
    return mainTimerSecondsElapsed() <= 300;
}