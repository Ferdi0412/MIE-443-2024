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
 *
 * Try to apply Lowe ratio test (referred to many times in OpenCV documentation)
*/

#ifndef FEATURE_SEARCHER_HPP
#define FEATURE_SEARCHER_HPP

// Project dependencies
#include <boxes.h>

// Base OpenCV dependencies
#include <opencv2/core.hpp>
#include <image_transport/image_transport.h>
// #include <cv.h>
// #include <cv_bridge/cv_bridge.h>

// OpenCV feature search/match dependencies
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

// Standard dependencies
#include <vector>  // std::vector
#include <utility> // std::pair

enum FeatureDetector {
    ORB,
    SURF,
    AKAZE
};

enum FeatureMatcher {
    BRUTE_FORCE,
    FLANN
};

class FeatureSearcher {
    private:
        // Searcher/matcher setup
        cv::Ptr<cv::DescriptorMatcher> feature_matcher;
        cv::Ptr<cv::Feature2D>         feature_detector;

        // Setup-dependent
        double match_threshold         = 1;    // How many of the features need to be matched with the template
        double lowe_threshold          = 0.7;  // The filter value for the Lowe's ratio test filter
        bool   apply_lowe_filter       = true; // Whether the Lowe's ratio test filter will be used or not

        // Box/target/template features
        std::vector<cv::Mat>                   template_descriptors;
        std::vector<std::vector<cv::KeyPoint>> template_keypoints;

        // Latest iteration features
        cv::Mat                                latest_img_descriptor;
        std::vector<cv::KeyPoint>              latest_img_keypoints;
        std::vector<cv::DMatch>                latest_matched_features;
        std::vector<std::vector<cv::DMatch>>   latest_knn_matched_features;



    private:
        // Functions for working with constructor
        void createFeatureDetector( FeatureDetector feature_detector_ );
        void createFeatureMatcher( FeatureMatcher feature_matcher_ );
        void initializeTemplates( const Boxes& boxes );

        // Methods for working with images
        cv::Mat makeGrayscale( const cv::Mat& img );

        // Methods for matching
        std::vector<cv::DMatch> applyLoweFilter( const std::vector<std::vector<cv::DMatch>>& all_matches );



    public:
        /* === CONSTRUCTORS/DESTRUCTORS === */
        /**
         * FeatureSearcher constructor
         *
         * @param boxes -> Boxes class to configure templates for boxes to detect
         * @param feature_detector_ -> Any of { ORB, SURF, AKAZE } to use to detect features
         * @param feature_matcher_  -> Any of { BRUTE_FORCE, FLANN } to use to detect feature matches
         *
         * @example FeatureSearcher feature_searcher( boxes, SURF, BRUTE_FORCE ); // Will initialize SURF w/ BRUTE_FORCE
        */
        FeatureSearcher ( const Boxes& boxes, FeatureDetector feature_detector_, FeatureMatcher feature_matcher_ );

        /**
         * FeatureSearcher constructor
         *
         * @param boxes             -> Boxes class to configure templates for boxes to detect
         * @param feature_detector_ -> OpenCV Feature2D detector sub-class to use
         * @param feature_matcher_  -> OpenCV DescriptorMatcher sub-class to use
         *
         * @example FeatureSearcher feature_searcher( boxes, cv::xfeatures2d::SURF::create(), new BFMatcher() ); // Will initialize SURF w/ BRUTE_FORCE
        */
        FeatureSearcher ( const Boxes& boxes, cv::Ptr<cv::Feature2D> feature_detector_, cv::Ptr<cv::DescriptorMatcher> feature_matcher_);

        /**
         * ~FeatureSearcher destructor
         *
         * @note this should clear the feature_matcher and feature_detector from memory...
        */
        // ~FeatureSearcher( void );

        /* === SETTINGS === */
        /**
         * setLoweThreshold modifies the threshold to be used when working with Lowe's thresholds
         *
         * @param new_threshold the new threshold to set
        */
        void setLoweThreshold( double new_threshold );

        /**
         * useLoweFilter changes whether the class does or does not apply the Lowe's ratio test to the matches
         *
         * @param use_filter -> true will make the class use the Lowe filter, false will make class not use it
        */
        void useLoweFilter( bool use_filter );

        /**
         * setFeatureMatchThreshold will set what portion of the features of the template must match to be a "match"
         *
         * @param new_threshold the new threshold to set
        */
        void setFeatureMatchThreshold( double new_threshold );

        /* === IMAGE SEARCH === */
        /**
         * checkForTemplate returns True if the desired template is detected
         *
         * @param img          -> the current kinect image
         * @param template_no  -> the number from the Boxes class of the desired template
         * @param template_img -> the image of the desired box/template
         *
         * @returns true if the template is detected
        */
        bool checkForTemplate( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img );

        /**
         * drawImg returns an image with overlays displaying the detected features
         *
         * @param img          -> the current kinect image
         * @param template_no  -> the number from the Boxes class of the desired template
         * @param template_img -> the image of the desired box/template
         *
         * @returns the annotated image
        */
        cv::Mat drawImg( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img );
};

#endif
