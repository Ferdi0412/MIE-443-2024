/**
 * Exploring some methods of image processing, as provided by OpenCV, for feature extraction/detection.
 *
 * There are several available options, notably:
 * 1. Template Matching (very naive, does not appear to useful)
 * 2. Haar Cascades     (Not sure how this works/is used)
 * 3. Feature Matching  (Appears very useful)
 *
 * Top options found for Feature Detecting (before matching):
 * 1. SURF  -> https://docs.opencv.org/3.4/d7/d66/tutorial_feature_detection.html
 * 2. ORB   -> https://docs.opencv.org/3.4/dc/dc3/tutorial_py_matcher.html
 * 3. AKAZE -> https://docs.opencv.org/4.x/db/d70/tutorial_akaze_matching.html
 *
 * Options for feature matching:
 * 1. Brute-Force -> https://docs.opencv.org/3.4/d7/d66/tutorial_feature_detection.html
 * 2. FLANN       -> https://docs.opencv.org/3.4/d7/d66/tutorial_feature_detection.html
*/

#ifndef FERDI_FUNCTIONS_CPP
#define FERDI_FUNCTIONS_CPP

#include <imagePipeline.h>
#include <boxes.h>

#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>

#include <iostream>

#define FRACTION_TO_MATCH 0.5

namespace FeatureSearch {
    // https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
    enum   SearchMethod { UNINITIALIZED, ORB, SURF };
    static SearchMethod search_method = UNINITIALIZED;
    static float fraction_to_match    = FRACTION_TO_MATCH;

    static cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create();
    static cv::Ptr<cv::ORB>               orb = cv::ORB::create();
    static cv::Ptr<cv::DescriptorMatcher> flann_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

    static std::vector<cv::Mat>                   box_descriptors;
    static std::vector<std::vector<cv::KeyPoint>> box_keypoints;

    static cv::Mat                   latest_img_descriptor;
    static std::vector<cv::KeyPoint> latest_img_keypoints;
    static std::vector<cv::DMatch>   latest_matched_features;

    void detect_and_compute( const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints_target, cv::Mat& descriptor_target ) {
        switch (search_method) {
            case UNINITIALIZED {
                std::cout << "ERROR!!!\nsearch_method is UNINITIALIZED" << std::endl;
                exit(-1);
            }
            case ORB {
                orb->detectAndCompute(img, cv::Mat(), keypoints_target, descriptor_target);
                break;
            }
            case SURF {
                surf->detectAndCompute(img, cv::Mat(), keypoints_target, descriptor_target);
                break;
            }
            default {
                std::cout << "ERROR!!!\nSomething critically wrong in detect_and_compute" << std::endl;
                exit(-1);
            }
        }
    }

    cv::Mat makeGrayscale( const cv::Mat& img ) {
        cv::Mat output_img;
        cv::cvtColor(img, output_img, cv::COLOR_BGR2GRAY);
        return output_img;
    }

    void initialize( const Boxes& boxes, SearchMethod search_method_to_use ) {
        assert( search_method_to_use != UNINITIALIZED );
        search_method = search_method_to_use;

        for ( insigned int i = 0; i < boxes.templates.size(); i++ ) {
            cv::Mat                   template_descriptor, template_img;
            std::vector<cv::KeyPoint> template_keypoints;

            // Fetch image of box template, and assert that it is grayscale
            template_img = boxes.templates[i];
            assert(template_img.channels() == 1);

            // Update the descriptors and keypoints for template
            detect_and_compute(template_img, template_keypoints, template_descriptor);

            // Store keypoints and descriptors
            box_descriptors.push_back(template_descriptor);
            box_keypoints.push_back(template_keypoints);
        }
    }

    bool search_templates( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img ) {
        detect_and_compute( makeGrayscale(img), latest_img_keypoints, latest_img_descriptor );
        flann_matcher.match(box_descriptors[template_no], latest_img_descriptor, latest_matched_features);
        return latest_matched_features.size() > (fraction_to_match * box_keypoints[template_no].size());
    }

    cv::Mat draw_matches( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img ) {
        cv::Mat drawing;
        cv::drawMatches()
    }
}

#endif
