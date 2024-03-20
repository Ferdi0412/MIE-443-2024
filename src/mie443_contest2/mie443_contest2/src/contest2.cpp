#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

#include "opencv2/core.hpp"

#include <chrono>
#include <cmath>
#include <fstream>

#include "parin-functions.cpp"
#include <auxilliary.h>
#include "auxilliary.h"

#define WINDOW_NAME "Matched template"

bool try_match_image( size_t box_index, ImagePipeline& image_pipeline, Boxes& boxes  );

bool try_move_to_box( size_t box_index, bool try_range_of_positions = false );

void try_display_image( cv::Mat image_to_display, float sleep_duration = 1. );

bool not_timedout();

int main(int argc, char** argv) {

    /**
     * =============
     * === SETUP ===
     * =============
    */
    // Course-provided setup
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    Boxes boxes;
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    ImagePipeline imagePipeline(n);

    // Custom stuff
    initialize_boxes_navigation( n, boxes, robotPose ); // From axuilliary.h
    imagePipeline.setMatchFunction(&match_function);    // From imagePipleine.h
    initialize_feature_detector(boxes.templates);       // From parin-functions.cpp
    set_required_good_matches(95);                      // Minimum number of "good_matches" to be identified as a template
    SimplePose start_pose(robotPose);                   // Starting position
    mainTimerStart();                                   // Start timer

    cv::namedWindow(WINDOW_NAME);

    /**
     * ============
     * === MAIN ===
     * ============
    */
    bool first_run    = true;
    size_t fail_count = 0;
    while(ros::ok() && not_timedout() ) {
        // Update position and images
        ros::spinOnce();

        // If all boxes have been found, go home and exit the main loop...
        if ( all_found() ) {
            if ( check_for_plan(start_pose) )
                if (Navigation::moveToGoal(start_pose.x, start_pose.y, start_pose.phi)) {
                    std::cout << "\n\n===MOVED TO HOME ===\n";
                }

            break;
        }

        bool has_moved = false;

        // Go through each box we need to find
        for ( size_t i = 0; i < boxes.coords.size(); i++ ) {
            // Check for timeout...
            if ( !not_timedout() )
                break;

            // Skip any that have been found
            if ( has_been_found(i) )
                continue;

            // If box is reachable, try to detect which image
            if ( try_move_to_box(i, !first_run) ) {
                has_moved = true;
                // Run image processing
                if ( try_match_image(i, imagePipeline, boxes ) ) {
                    // Retrieve the latest template_id
                    int template_id = get_box_id(i);

                    // If the image is blank, try move a little backwards and try again...
                    if ( is_blank(template_id) ) {
                        std::cout << "BLANK - Moving backwards and checking again...";
                        move_robot_by( -0.1, 0 );
                        try_match_image(i, imagePipeline, boxes);
                        template_id = get_box_id(i);
                    }

                    // Display the results
                    try_display_image( get_image(imagePipeline, template_id));

                    }
                } else // If not try_match_image
                    std::cout << "\nFEATURE DETECT - Could not PROCESS box === {" << i << "} ===\n\n";
            } else // If not try_move_to_box
                std::cout << "\nCould not reach box === {" << i << "} ===\n\n";
        }

        // If the robot never moved to any valid box... It must be stuck... Try move away from any obstacles...
        if ( !has_moved && not_timedout() ) {
            std::cout << "\n\n=== !!! ROBOT STUCK !!! ===\n\n";

            // Keep track of consecutive stuck iterations...
            fail_count ++;

            // Handle various fail
            switch ( fail_count ) {
                case 1:
                    std::cout << "FAILED ONCE: Clearing Costmap...\n";
                    if ( !refresh_costmap() )
                        std::cout << "CLEAR COSTMAP FAILED!!!\n";
                    break;

                default:
                    std::cout << "FAILED MORE THAN ONCE: Resetting Pose Estimate and Clearing Costmap...\n";
                    if ( !move_pose_estimate( -0.05, 0 ) )
                        std::cout << "MOVE POSE ESTIMATE FAILED!!!\n";
                    if ( !refresh_costmap() )
                        std::cout << "CLEAR COSTMAP FAILED!!!\n";
                    break;
            }
        } else // Reset fail_count if not stuck on this iteration
            fail_count = 0;

        // After the first run, allow the robot to try other positions than the "default"
        first_run = false;
    }

    // After timeout....
    std::cout << "\n\n=== TEMPLATES FOUND ===\n";
    std::vector<int> found_boxes = get_box_ids();
    // Open the file for writing
    std::ofstream outputFile("output.txt");

    // Check if the file is opened successfully
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open output file!" << std::endl;
        return -1;
    }

    for ( size_t i = 0; i < found_boxes.size(); i++ ) {
        int template_id = found_boxes[i]; // IF -1, not identified, if template_id >= boxes.templates.size() - no template
        float x = boxes.coords[i][0];
        float y = boxes.coords[i][1];
        float phi = boxes.coords[i][2];

        // Just printing stuff...
        std::cout << i << " := " << found_boxes[i] << std::endl;

        // Write data to the file
        outputFile << "Box " << i << ": ";
        if (template_id >= 0 && template_id < boxes.templates.size()) {
            outputFile << " | Template Id: " << template_id ;
        } else if (template_id >= boxes.templates.size()){
            outputFile << " | Template Id (Blank Image): " << template_id;
        } else
            outputFile << " | Template not identified";
        outputFile << " | Coordinates: (x=" << x << ", y=" << y << ", phi=" << phi << ")" << std::endl;
    }

    // Close the file
    outputFile.close();

    // Close the window displaying the images
    cv::destroyWindow(WINDOW_NAME);

    // To add... store to file
    std::cout << "\n\nPROGRAM END\n";
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


bool try_move_to_box( size_t box_index, bool try_range_of_positions ) {
    SimplePose facing_box = location_facing_box( box_index ); // Start with default position facing box

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

    // If try_range_of_positions and default point not found... Test other positions...
    if ( try_range_of_positions ) {
        // Start iterating through various distances facing the position head-on
        for ( float distance = 0.05; distance <= 0.55; distance += 0.1 ) {
            facing_box = location_facing_box( box_index, distance );
            if ( check_for_plan( facing_box ) ) {
                std::cout << "Moving using distance of " << distance << " m\n";
                bool success = Navigation::moveToGoal( facing_box.x, facing_box.y, facing_box.phi );
                ros::spinOnce();
                return success;
            }
        }

        // Start iteration through combination of angle-offset and distances
        for ( float angle = -degree_2_radian(20); angle <= degree_2_radian(20); angle += degree_2_radian(10) ) {
            for ( float distance = 0.05; distance <= 0.5; distance += 0.15 ) {
                facing_box = location_facing_box( box_index, distance, angle );
                if ( check_for_plan( facing_box ) ) {
                    std::cout << "Moving using angle of " << angle << " and distance of " << distance << " m\n";
                    bool success = Navigation::moveToGoal( facing_box.x, facing_box.y, facing_box.phi );
                    ros::spinOnce();
                    return success;
                }
            }
        }
    } else // If not try_range_of_positions, immediately return false...
        return false;
}



bool not_timedout() {
    return mainTimerSecondsElapsed() <= 300;
}

void try_display_image( cv::Mat image_to_display, float sleep_duration ) {
    // Prevent bugs...
    if ( image_to_display.empty() )
        return;
    try {
        cv::imshow(WINDOW_NAME, make_grayscale_copy(image_to_display));
        cv::waitKey(1000);
        ros::Duration(sleep_duration).sleep();
    } except ( cv::Exception& exc ) {
        ;
    }
}
