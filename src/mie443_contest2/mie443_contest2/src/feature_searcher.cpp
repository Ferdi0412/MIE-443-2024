/**
 * Not ready yet!
*/
#include <feature_searcher.h>

#include <iostream>

/* === PRIVATE functions === */
void FeatureSearcher::createFeatureDetector( FeatureDetector feature_detector_ ) {
    switch ( feature_detector_ ) {
        case ORB:
            feature_detector = cv::ORB::create();
            break;

        case SURF:
            feature_detector = cv::xfeatures2d::SURF::create();
            break;

        case AKAZE:
            feature_detector = cv::AKAZE::create();
            break;

        default:
            std::cout << "Invalid feature_detector_" << std::endl;
            exit(-1);
    }
}

void FeatureSearcher::createFeatureMatcher( FeatureMatcher feature_matcher_ ) {
    switch( feature_matcher_ ) {
        case BRUTE_FORCE:
            feature_matcher = cv::BFMatcher::create();
            break;

        case FLANN:
            feature_matcher = cv::FlannBasedMatcher::create();
            break;

        default:
            std::cout << "Invalid feature_matcher_" << std::endl;
            exit(-1);
    }
}

void FeatureSearcher::initializeTemplates( const Boxes& boxes ) {
    // NOTE: Must run after feature_detector is initialized...
    // assumes that template_descriptors and template_keypoints are empty...
    for ( unsigned int i = 0; i < boxes.templates.size(); i++ ) {
        cv::Mat                   box_descriptor, template_image;
        std::vector<cv::KeyPoint> box_keypoints;

        template_image = makeGrayscale(boxes.templates[i]);

        feature_detector->detectAndCompute( template_image, cv::Mat(), box_keypoints, box_descriptor );

        template_descriptors.push_back( box_descriptor );
        template_keypoints.push_back( box_keypoints );
    }
}

cv::Mat FeatureSearcher::makeGrayscale( const cv::Mat& img ) {
    cv::Mat output_img;
    cv::cvtColor(img, output_img, cv::COLOR_BGR2GRAY);
    return output_img;
}

std::vector<cv::DMatch> FeatureSearcher::applyLoweFilter( const std::vector<std::vector<cv::DMatch>>& all_matches ) {
    // Implemented following https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
    // TODO: Read more about Lowe's filter
    std::vector<cv::DMatch> good_matches;
    for ( size_t i = 0; i < all_matches.size(); i++ ) {
        if ( all_matches[i][0].distance < lowe_threshold * all_matches[i][1].distance )
            good_matches.push_back(all_matches[i][0]);
    }
    return good_matches;
}



/* === CONSTRUCTORS/DESTRUCTORS === */
FeatureSearcher::FeatureSearcher( const Boxes& boxes, FeatureDetector feature_detector_, FeatureMatcher feature_matcher_ ) {
    createFeatureDetector(feature_detector_);
    createFeatureMatcher(feature_matcher_);
    initializeTemplates( boxes );
}

FeatureSearcher::FeatureSearcher( const Boxes& boxes, cv::Ptr<cv::Feature2D> feature_detector_, cv::Ptr<cv::DescriptorMatcher> feature_matcher_ ) {
    feature_detector = feature_detector_;
    feature_matcher  = feature_matcher_;
    initializeTemplates( boxes );
}


/* === SETTINGS === */
void FeatureSearcher::setLoweThreshold( double new_threshold ) {
    lowe_threshold = new_threshold;
}

void FeatureSearcher::useLoweFilter( bool use_filter ) {
    apply_lowe_filter = use_filter;
}

void FeatureSearcher::setFeatureMatchThreshold( double new_threshold ) {
    match_threshold = new_threshold;
}



/* === IMAGE SEARCH === */
bool FeatureSearcher::checkForTemplate( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img ) {
    feature_detector->detectAndCompute( makeGrayscale(img), cv::Mat(), latest_img_keypoints, latest_img_descriptor );
    if ( apply_lowe_filter ) {
        feature_matcher->knnMatch( template_descriptors[template_no], latest_img_descriptor, latest_knn_matched_features, 2);
        latest_matched_features = applyLoweFilter(latest_knn_matched_features);
    }
    else
        feature_matcher->match( template_descriptors[template_no], latest_img_descriptor, latest_matched_features );
    return latest_matched_features.size() > match_threshold; // TODO: Find a useful threshold value...
}

cv::Mat FeatureSearcher::drawImg( const cv::Mat& img, unsigned int template_no, const cv::Mat& template_img ) {
    // TODO: You need to find out how to show the matched features...
    // A) By updating the keypoints: https://docs.opencv.org/4.x/db/d70/tutorial_akaze_matching.html
    // B) By updating the DMatches : https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
    // Both of the above must happen in applyLoweFilter...

    cv::Mat drawn_img;
    cv::drawMatches( template_img, template_keypoints[template_no], img, latest_img_keypoints, latest_matched_features, drawn_img );
    return drawn_img;
}
