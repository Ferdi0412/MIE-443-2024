#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <auxilliary.h>
#include <chrono>

#include "parin-functions.cpp"
#include "auxilliary.h"


bool try_match_image( size_t box_index, ImagePipeline& image_pipeline, Boxes& boxes  );

bool try_move_to_box( size_t box_index, bool try_range_of_positions = false );

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
    set_required_good_matches(60);                      // Minimum number of "good_matches" to be identified as a template
    SimplePose start_pose(robotPose);                   // Starting position
    mainTimerStart();                                   // Start timer


    /**
     * ============
     * === MAIN ===
     * ============
    */
    bool first_run = true;
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
                if ( try_match_image(i, imagePipeline, boxes ) ) {
                    std::cout << "\nDisplaying matched image..." << std::endl;
                    int template_id;
                    if ( (template_id = get_box_id(i)) > 0 ) {
                        cv::imshow("Template matched", boxes.templates[template_id]);
                        cv::waitKey(10);
                        ros::Duration(1.).sleep();
                        continue;
                    }
                } else
                    std::cout << "\nCould not process box === {" << i << "} ===\n\n";
            } else
                std::cout << "\nCould not reach box === {" << i << "} ===\n\n";
        }
        first_run = false;
    }

    std::cout << "\n\n=== TEMPLATES FOUND ===\n";
    std::vector<int> found_boxes = get_box_ids();
    for ( size_t i = 0; i < found_boxes.size(); i++ ) {
        std::cout << i << " := " << found_boxes[i] << std::endl;
    }

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
                bool success = Navigation::moveToGoal( facing_box.x, facing_box.y, facing_box.phi );
                ros::spinOnce();
                return success;
            }
        }

        // Start iteration through combination of angle-offset and distances
        for ( float angle = -degree_2_radian(20); angle <= degree_2_radian(20); angle += degree_2_radian(10) ) {
            for ( float distance = 0.05; distance <= 0.5; distance += 0.15 ) {
                if ( check_for_plan( facing_box ) ) {
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
