#ifndef FERDI_FUNCTIONS_CPP
#define FERDI_FUNCTIONS_CPP

#include <imagePipeline.h>

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

#endif
