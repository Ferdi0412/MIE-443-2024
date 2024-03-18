#ifndef PARIN_FUNCTIONS_CPP
#define PARIN_FUNCTIONS_CPP

#include <iostream>

#include "ros/ros.h"

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <imagePipeline.h>
# include "boxes.h"

using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

// You need to initailize the featured detecotr (to identify features) and matcher (to compare images)
static cv::Ptr<cv::Feature2D>         feature_detector;
static cv::Ptr<cv::DescriptorMatcher> feature_matcher;

// For each template, you will get a vector of <KeyPoint>s, and a <Mat> of descriptors...
static std::vector<std::vector<cv::KeyPoint>> keypoints_all_templates;
static std::vector<cv::Mat>                   descriptors_all_templates;
static std::vector<cv::Mat>                   grayscale_templates;

static float ratio_thresh = 0.75;

static bool draw_matches = false;



/**
 * make_grayscale_copy returns a grayscale copy of the input image
*/
cv::Mat make_grayscale_copy( const cv::Mat& non_grayscale ) {
    // Enforce grayscale transform
    cv::Mat grayscale_img;
    // Check what colors are in the inputted image...
    if ( non_grayscale.channels() == 1 )
        return non_grayscale.clone(); // Already grayscale
    else if ( non_grayscale.channels() == 3 )
        cv::cvtColor( non_grayscale, grayscale_img, cv::COLOR_BGR2GRAY );
    else if ( non_grayscale.channels() == 4 )
        cv::cvtColor( non_grayscale, grayscale_img, cv::COLOR_BGRA2GRAY );
    else
        std::cout << "MAKE_GRAYSCALE_COPY: Unrecognized color channels!";
    return grayscale_img;
}



/**
 * initialize_feature_detector must be run before match_function at least once
*/
void initialize_feature_detector( const std::vector<cv::Mat>& box_templates, int min_hessian = 400, bool draw_all_matches = false ) {
    feature_detector = SURF::create( min_hessian ); // ORB::create() or AKAZE::create();
    // feature_matcher  = FlannBasedMatcher::create();
    feature_matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);

    draw_matches = draw_all_matches;

    for ( size_t i = 0; i < box_templates.size(); i++ ) {
        std::vector<cv::KeyPoint> keypoints_this_template;
        cv::Mat                   descriptors_this_template;
        cv::Mat img_template = make_grayscale_copy(box_templates[i]);

        // Setup descriptors and keypoints...
        feature_detector->detectAndCompute( img_template, cv::Mat(), keypoints_this_template, descriptors_this_template );

        // Store them so they can be accessed in match_function
        keypoints_all_templates.push_back( keypoints_this_template );
        descriptors_all_templates.push_back( descriptors_this_template );
        grayscale_templates.push_back( img_template );
    }
}


/**
 * match_function will return the best match (if available)... To consider: adding a min_good_matches
*/
int match_function( const cv::Mat& img, const std::vector<cv::Mat>& box_templates, int min_good_matches = 0 ) {
    // You only need one of each for the input image
    std::vector<cv::KeyPoint> keypoints_scene;
    cv::Mat                   descriptors_scene;

    int max_good_matches = min_good_matches;
    int max_index = -1;

    /**
     * If using knnMatch -> The method suggested in the course:
     * You have many templates, for each you will get a 2-d vector of <DMatch>
     * They are 2D because they containt N-best sets of matches, so you need a vector of sets of matches...
     * So to have many comparisons (with many templates), you need a 3-D vector of <DMatch> to use this method.
     * HOWEVER, we filter down to only the "good" matches, so the vector we care only about the good matches (1D per template)
     * 2D for entire thing
     *
     * If using match -> Simpler, but not the suggested one:
     * You have many templates, for each you will receive a vector of <DMatch>, so you need a 2-D vector of matches
    */
    std::vector<std::vector<cv::DMatch>> all_good_matches;

    // Safeguard against an invalid img -> could lead to some weird errors!!!
    if ( img.empty() ) {
        std::cout << "Could not open or find the image!\n" << endl;
        return -1;
    }

    // You also need a grayscale copy of the input image
    cv::Mat img_scene = make_grayscale_copy(img);

    // finding keypoints in both the object image and the scene image
    // There is the option to input a "filter" -> noArray means do not add an extra filter...
    feature_detector->detectAndCompute( img_scene, cv::noArray(), keypoints_scene, descriptors_scene );

    // Iterate through all templates for matching...
    for ( size_t i = 0; i < box_templates.size(); i++ ) {
        // For this template [i], we need to store the knn matches somewhere... -> 2D array if using knnMatch
        std::vector<std::vector<DMatch>> knn_matches;
        // If using knnMatch, you also need to filter the "good" matches
        std::vector<DMatch>              good_matches;
        // Get the template (in grayscale) that we are matching
        cv::Mat                          template_img = grayscale_templates[i];

        // Extract descriptors and keypoints for teamplate from the lists...
        std::vector<cv::KeyPoint> keypoints_template   = keypoints_all_templates[i];
        cv::Mat                   descriptors_template = descriptors_all_templates[i];

        // Match template and scene features
        feature_matcher->knnMatch( descriptors_template, descriptors_scene, knn_matches, 2 );
        // feature_matcher->match(...)

        // If using knnMatch... You use Lowe's ratio test to filter the best fitted sets of matches
        for ( size_t j = 0; j < knn_matches.size(); j++ ) {
            if (knn_matches[j][0].distance < ratio_thresh*knn_matches[j][1].distance) {
                good_matches.push_back(knn_matches[j][0]);
            }
        }
        all_good_matches.push_back( good_matches );

        // Get average distance from good_matches
        // float mean_distance = 0;
        // for ( size_t j = 0; j < good_matches.size(); j++ ) {
        //     mean_distance += good_matches[j].distance / good_matches.size();
        // }

        if ( draw_matches ) {
            // Drawing a match for a single template and the inputted scene
            cv::Mat img_matches;
            cv::drawMatches( template_img, keypoints_template, img_scene, keypoints_scene, good_matches, img_matches,
                            // colors of matches in either image [x2], empty string - label matches I think, Filter away unmatched points
                            Scalar::all(-1), Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

            // Store positions of the "good" matches -> where each matched feature is in either image, to calculate
            // translation/rotation in the scene - position in scene of template
            std::vector<cv::Point2f> feature_positions_template, feature_positions_scene;
            for ( size_t j = 0; j < good_matches.size(); j++ ) {
            feature_positions_template.push_back( keypoints_template[ good_matches[j].queryIdx ].pt );
            feature_positions_scene.push_back( keypoints_template[ good_matches[j].trainIdx ].pt );
            }

            // Transformation from template to scene
            cv::Mat homography = cv::findHomography( feature_positions_template, feature_positions_scene, cv::RANSAC );

            // Get corners of object in template
            std::vector<Point2f> template_corners(4);
            template_corners[0] = Point2f(0, 0);
            template_corners[1] = Point2f( (float)template_img.cols, 0 );
            template_corners[2] = Point2f( (float)template_img.cols, (float)template_img.rows );
            template_corners[3] = Point2f( 0, (float)template_img.rows );

            // Get corners of scene image
            std::vector<Point2f> scene_corners(4);
            cv::perspectiveTransform( template_corners, scene_corners, homography );

            // Draw the outline of the matched image...
            Point2f template_cols((float)template_img.cols, 0);
            line(img_matches, scene_corners[0] + template_cols, scene_corners[1] + template_cols, Scalar(0,255,0), 4);
            line(img_matches, scene_corners[1] + template_cols, scene_corners[2] + template_cols, Scalar(0,255,0), 4);
            line(img_matches, scene_corners[2] + template_cols, scene_corners[3] + template_cols, Scalar(0,255,0), 4);
            line(img_matches, scene_corners[3] + template_cols, scene_corners[0] + template_cols, Scalar(0,255,0), 4);

            // Show matches detected
            imshow("Good matches & object detection", img_matches);
            waitKey(100);
            ros::Duration(1).sleep();
        } // ~ draw_matches
    }


    // Iterate through all sets of good matches
    for (size_t i = 0; i < all_good_matches.size(); i++) {
        int num_good_matches = all_good_matches[i].size();
        if (num_good_matches > max_good_matches) {
            max_good_matches = num_good_matches;
            max_index = i;
        }
    }
    return max_index;
}

#endif // ~ PARIN_FUNCTIONS_CPP
