#ifndef FEATURE_DETECT_H
#define FEATURE_DETECT_H

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

class DetectedFeatures {
    public:
        cv::Mat                   descriptors;
        std::vector<cv::KeyPoint> keypoints;
};

class GoodMatches {
    public:
        std::vector<cv::KeyPoint> good_keypoint_template;
        std::vector<cv::KeyPoint> good_keypoint_img;

        DetectedFeatures          all_features_img;

        size_t count_matches( );
}

class FeatureMatcher {
    private:
        // OpenCV Classes and ID storage
        int                            _detector_type;
        cv::Ptr<cv::Feature2D>         _feature_detector;
        cv::Ptr<cv::DescriptorMatcher> _feature_matcher;

        // Template storage
        cv::Mat                        _template_color;
        cv::Mat                        _template_grayscale;

        // Template feature storage
        std::vector<cv::KeyPoint>      _keypoints_template;
        cv::Mat                        _descriptors_template;

        // Private functions
        FeatureMatcher( std::string file_name, cv::Ptr<cv::Feature2D> feature_detector_, cv::Ptr<cv::DescriptorMatcher> feature_matcher_, int detector_type_ );
        DetectedFeatures detect_and_compute( const cv::Mat& img );
        GoodMatches match( const DetectedFeatures& img_features, double lowe_ratio = 0.75 );

    public:
        /**
         * There is no public constructor - intialize as follows:
         *
         * FeatureMatcher feature_matcher = FeatureMatcher::get_SURF( "template.jpg" );
        */
        static FeatureMatcher get_SURF(  std::string template_name, unsigned int min_hessian = 400 );
        static FeatureMatcher get_ORB(   std::string template_name );
        static FeatureMatcher get_AKAZE( std::string template_name );

        /**
         * to_grayscale - returns a grayscale copy of img_to_copy
        */
        static cv::Mat to_grayscale( const cv::Mat& img_to_copy );

        /**
         * match_features - returns GoodMatches object
        */
        GoodMatches match_features( const cv::Mat& img );

        /**
         * get_template_descriptors - returns descriptors for template
        */
        const cv::Mat& get_template_descriptors( );

        /**
         * get_template_keypoints - returns keypoints for template
        */
        const std::vector<cv::KeyPoint>& get_template_keypoints( );

        /**
         * get_template - returns template image
        */
        cv::Mat get_template( );
};

void initialize_features( std::string file_name );

bool match_image( cv::Mat img );

#endif // ~ FEATURE_DETECT_H
