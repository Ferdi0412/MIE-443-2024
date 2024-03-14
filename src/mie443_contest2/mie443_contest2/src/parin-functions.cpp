#include <iostream>
#include "opencv2/core.hpp"
#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#ifndef PARIN_FUNCTIONS_CPP
#define PARIN_FUNCTIONS_CPP

#include <imagePipeline.h>
# include "boxes.h"

using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

const char* keys =
"{ help h | | Print help message. }"
"{ input1 | box.png | Path to input image 1. }"
"{ input2 | box_in_scene.png | Path to input image 2. }";

static std::vector<cv::Mat> template_descriptors;
static std::vector<std::vector<cv::KeyPoint>> template_keypoints;

int match_function( const cv::Mat& argc, const std::vector<cv::Mat>& argv ) {
    CommandLineParser parser( argc, argv, keys );
    Mat img_object = imread(parser.get<String>("input1"), IMREAD_GRAYSCALE );
    Mat img_scene = imread(parser.get<String>("input2"), IMREAD_GRAYSCALE );
    if ( img_object.empty() || img_scene.empty() ) {
        cout << "Could not open or find the image!\n" << endl;
        parser.printMessage();
        return -1;
    }


    // finding keypoints in both the object image and the scene image
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    detector->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object );
    detector->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );


    // two keypoints match is based on the distance between them
    Ptr<DescriptorMatcher> matcher =
    DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );
    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.75f;

    // first and second nearest neighbours between the images
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh*knn_matches[i][1].distance)
            {
            good_matches.push_back(knn_matches[i][0]);
            }
    }

    // Outlier Rejection and Drawing Matches
    Mat img_matches;
    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, good_matches,
    img_matches, Scalar::all(-1),
    Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }


    // Transformation that best maps the keypoints in the object to the keypoints in the scene
    Mat H = findHomography( obj, scene, RANSAC );
    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = Point2f(0, 0);
    obj_corners[1] = Point2f( (float)img_object.cols, 0 );
    obj_corners[2] = Point2f( (float)img_object.cols, (float)img_object.rows );
    obj_corners[3] = Point2f( 0, (float)img_object.rows );
    std::vector<Point2f> scene_corners(4);
    perspectiveTransform( obj_corners, scene_corners, H);

    //skewed box should be drawn around where the object is located in the scene image
    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f((float)img_object.cols, 0),
    scene_corners[1] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f((float)img_object.cols, 0),
    scene_corners[2] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f((float)img_object.cols, 0),
    scene_corners[3] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f((float)img_object.cols, 0),
    scene_corners[0] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    //-- Show detected matches
    imshow("Good Matches & Object detection", img_matches );
    waitKey();
    return 0;
}

