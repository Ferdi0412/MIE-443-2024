#ifndef FERDI_FUNCTIONS_CPP
#define FERDI_FUNCTIONS_CPP

#include <imagePipeline.h>
#include <boxes.h>

#include <vector>

#include <iostream>

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

namespace SurfSearch {
    static cv::Ptr<cv::SURF> surf = cv::SURF::create();
    static cv::Ptr<cv::FlassBasedMatcher> surf_matcher = cv::FlannBasedMatcher::create();
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
