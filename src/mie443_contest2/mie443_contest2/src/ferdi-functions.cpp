#ifndef FERDI_FUNCTIONS_CPP
#define FERDI_FUNCTIONS_CPP

#include <imagePipeline.h>
#include <boxes.h>

#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>

#include <iostream>

#define FRACTION_TO_MATCH 0.5

namespace Ferdi {
    static cv::Mat latest_res, latest_locations;

    /**
     * Ferdi::makeGrayscale returns a grayscale version of a 3 channel image
     *
     * @param img the image to return a grayscale clone of
     * @returns grayscale clone of input
    */
    cv::Mat makeGrayscale( const cv::Mat& img ) {
        cv::Mat output_img;
        cv::cvtColor(img, output_img, cv::COLOR_BGR2GRAY);
        return output_img;
    }

    /**
     * Ferdi::match_templates tries to match img to template_img
     *
     * @implements the cv::matchTemplate; cv::findNonZero methods to search for template_img matches
     * @note       Ferdi::latest_res; Ferdi::latest_locations [both PRIVATE] to store search results
     *
     * @param img          the image to match
     * @param template_img the template to match
     * @param match_method the error: no matching function for call to ‘cv::Mat::begin()’
         for ( cv::Point p : latest_locations )method to use in matching
     * @param threshold    the threshold to use in matching
     *
     * @returns a count of the matches
    */
    unsigned int match_templates( const cv::Mat& img, const cv::Mat& template_img, int match_method, double threshold ) {
        // std::cout << "Depth img: " << img.depth() << std::endl << "Depth tpl: " << template_img.depth() << std::endl << "Channels img: " << img.channels() << std::endl << "Channels tpl: " << template_img.channels() << std::endl;
        cv::matchTemplate( makeGrayscale(img), template_img, latest_res, match_method );
        std::cout << latest_res;
        cv::findNonZero( latest_res > threshold, latest_locations );
        return latest_locations.total();
    }


    /**
     * Ferdi::search_templates tries to match img to template_img
     *
     * @param img the image to match
     * @param template_img the template to match
     *
     * @a Uses the <TM_CCOEFF_NORMED> match method
     * @a Uses a threshold value of <0.5> to match
    */
    bool search_templates( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img ) {
        return match_templates( img, template_img, cv::TM_CCOEFF_NORMED, 0.5 ) > 0;
    }

    /**
     * Ferdi::draw_templates tries to draw rectangles around each detected instance of template_img
     *
     * @note  Assumes that search_templates has already been run on it...
     *
     * @param img the image to match
     * @param template_img the template to match
    */
    cv::Mat draw_templates( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img) {
        cv::Mat drawing = img.clone();
        std::cout << latest_locations;
        // for ( cv::Point p : latest_locations )
        //     cv::rectangle( drawing, p, cv::Point(p.x + template_img.cols, p.y + template_img.rows), cv::Scalar(0, 0, 255), 4 );
        return drawing;
    }
}

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

// ORB appears to be unavailable for the OpenCV release we are using...
// namespace OrbSearch {
//     static cv::Ptr<cv::ORB>      orb = cv::ORB::create();
//     static cv::FlannBasedMatcher orb_matcher;

//     static std::vector<cv::Mat>                   box_descriptors;
//     static std::vector<std::vector<cv::KeyPoint>> box_keypoints;

//     static cv::Mat                   latest_img_descriptor;
//     static std::vector<cv::KeyPoint> latest_img_keypoints;
//     static std::vector<cv::DMatch>   latest_matches;

//     bool initialize(const Boxes& boxes) {
//         for ( unsigned int i = 0; i < boxes.templates.size(); i++ ) {
//             cv::Mat template_descriptor, template_img;
//             std::vector<cv::KeyPoint> template_keypoints;
//             template_img = boxes.templates[i];
//             assert(template_img.channels() == 1);
//             orb->detectAndCompute( template_img, cv::Mat(), template_keypoints, template_descriptor );
//             box_descriptors.push_back( template_descriptor );
//             box_keypoints.push_back( template_keypoints );
//         }
//     }

//     cv::Mat makeGrayscale( const cv::Mat& img ) {
//         cv::Mat output_img;
//         cv::cvtColor(img, output_img, cv::COLOR_BGR2GRAY);
//         return output_img;
//     }

//     bool search_templates( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img ) {
//         orb->detectAndCompute( makeGrayscale(img), cv::Mat(), latest_img_keypoints, latest_img_descriptor );
//         orb_matcher.match(box_descriptors[template_no], latest_img_descriptor, latest_matches);
//         return latest_matches.size() > 0;
//     }

//     cv::Mat draw_matches( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img ) {
//         cv::Mat drawing;
//         cv::drawMatches( template_img, box_keypoints[template_no], makeGrayscale(img), latest_img_keypoints, matches, drawing );
//         return drawing;
//     }


// }

#endif
