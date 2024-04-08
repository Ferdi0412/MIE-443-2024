#ifndef FEATURE_DETECT_CPP
#define FEATURE_DETECT_CPP

#include "../feature_detect.h"
#include <exception>


#define _SURF_ID 1
#define _NON_SURF_ID 0



size_t GoodMatches::count_matches( ) {
    return keypoint_img.size();
}



FeatureMatcher::FeatureMatcher( std::string                    file_name,
                                cv::Ptr<cv::Feature2D>         feature_detector_,
                                cv::Ptr<cv::DescriptorMatcher> feature_matcher_,
                                int detector_type_ ) :
        _detector_type(    detector_type_    ),
        _feautre_detector( feature_detector_ ),
        _feature_matcher(  feature_matcher_  )
{
    std::string template_file = ros::package::getPath( "mie443_contest3" ) + "/images/" + file_name;
    _template_color = cv::imread( template_file, cv::IMREAD_COLOR );
    if ( _template_color.empty() ) {
        std::string err_msg = "=== ERROR: FeatureMatcher could not open file " + template_file;
        throw std::exception( err_msg );
    }
    _template_grayscale   = to_grayscale( template_img );

    DetectedFeatures template_features = detect_and_compute( _template_color );

    _keypoints_template   = template_features.keypoints;
    _descriptors_template = template_features.descriptors;
}

DetectedFeatures FeatureMatcher::detect_and_compute( const cv::Mat& img ) {
    DetectedFeatures features_detected;
    _feature_detector->detect_and_compute( img, cv::Mat(), features_detected.keypoints, features_detected.descriptors );
    return features_detected;
}



GoodMatches FeatureMatcher::match( const DetectedFeatures& img_features, double lowe_ratio ) {
    // if ( _detector_type == _SURF_ID ) {
    std::vector<cv::DMatch>              good_matches;
    std::vector<std::vector<cv::DMatch>> knn_matches;
    GoodMatches                          good_match_obj;

    // Allow the user to retrieve all descriptors and keypoints for the input....
    good_match_obj.all_feautures_img = img_features;

    // Run the match function....
    _feature_matcher->knnMatch( _descriptors_template, img_features.descriptors, knn_matches, 2 );

    // Apply Lowe's filter
    for ( size_t i = 0; i < knn_matches.size(); i++ ) {
        if ( knn_matches[i][0].distance < (lowe_ratio * knn_matches[i][1].distance) )
            good_matches.push_back(knn_matches[i][0]);
    }

    // Retrieve the corresponding keypoints as filtered above
    for ( size_t i = 0; i < good_matches.size(); i++ ) {
        good_match_obj.keypoint_template.push_back( _keypoints_template[good_matches[i].queryIdx]    );
        good_match_obj.keypoint_img.push_back(      img_features.keypoints[good_matches[i].trainIdx] );
    }

    // Return the result...
    return good_match_obj;
    // }
}



FeatureMatcher FeatureMatcher::get_SURF( std::string template_name, unsigned int min_hessian ) {
    return FeatureMatcher( template_name, cv::xfeatures2d::SURF::create(min_hessian), cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED) );
}



FeatureMatcher FeatureMatcher::get_ORB( std::string template_name ) {
    return FeatureMatcher( template_name, cv::ORB::create(), cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING));
}



FeatureMatcher FeatureMatcher::get_AKAZE( std::string template_name ) {
    return FeatureMatcher( template_name, cv::AKAZE::create(), cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING));
}




cv::Mat FeatureMatcher::to_grayscale( const cv::Mat& img_to_copy ) {
    // Enforce grayscale transform
    cv::Mat grayscale_img;
    // Check what colors are in the inputted image...
    switch( img_to_copy.channels() ) {
        case 1:
            return img_to_copy.clone(); // Already grayscale

        case 3:
            cv::cvtColor( img_to_copy, grayscale_img, cv::COLOR_BGR2GRAY );
            break;

        case 4:
            cv::cvtColor( img_to_copy, grayscale_img, cv::COLOR_BGRA2GRAY );
            break;

        default:
            std::cout << "FeatureMatcher::to_grayscale: Unrecognized color channels!";
            break;
    }
    return grayscale_img;
}



GoodMatches FeatureMatcher::match_features( const cv::Mat& img ) {
    GoodMatches good_matches;
    if ( img.empty() )
        return good_matches;
    DetectedFeatures features = detect_and_compute( img );
    good_matches = match( features );
    return good_matches;
}



const cv::Mat& get_template_descriptors( ) {
    return _descriptors_template;
}



const std::vector<cv::KeyPoint>& get_template_keypoints( ) {
    return _keypoints_template;
}



cv::Mat get_template( ) {
    return _template_color;
}



#endif // ~ FEATURE_DETECT_CPP
